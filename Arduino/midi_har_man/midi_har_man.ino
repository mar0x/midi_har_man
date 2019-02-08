
#if !defined(__AVR_ATtiny84__)
#error Only for ATtiny84
#endif

#include <Arduino.h>

#if !defined(PCICR)
#define PCICR GIMSK
#endif

// enable debug output
// #define DEBUG 1

// output debug via TWI
// #define DEBUG_TWI 1

// enable LED input debugging
// #define DEBUG_LED 1

// #define DEBUG_LOOP 1

#include "soft_serial.h"

soft_serial<9, 6, 31250> midi_in;

#if defined(DEBUG)

#if defined(DEBUG_TWI)
#include "twi.h"

using twi_t = twi< artl::digital_out<6>, artl::digital_out<5> >;
twi_t debug_twi;
#define DEBUG_PRINT debug_twi
#else
#define DEBUG_PRINT midi_in
#endif

#endif

#include "artl/digital_in.h"
#include "artl/digital_out.h"
#include "artl/digital_pin.h"
#include "artl/pin_change_int.h"
#include "artl/timer.h"
#include "artl/yield.h"

#include "artl/bit_order.h"
#include "artl/piso.h"

#include "debug.h"
#include "midi_cmd.h"

#include "cs.h"

using btn = artl::digital_pin<10>;

using led_in_0 = artl::digital_in<0>;
using led_in_1 = artl::digital_in<1>;
using led_in_2 = artl::digital_in<2>;
using led_in_3 = artl::digital_in<3>;

using led_0_int = artl::pin_change_int< led_in_0::pin >;
using led_1_int = artl::pin_change_int< led_in_1::pin >;
using led_2_int = artl::pin_change_int< led_in_2::pin >;
using led_3_int = artl::pin_change_int< led_in_3::pin >;

enum {
    MAX_LED = 4,
    PUSH_TIME = 20, // ms
    PUSH_PAUSE = 20, // ms
    MIN_EXCLUSIVE_TIME = 50, // ms
    MAX_CURRENT_DELAY = 1000, // ms
};

unsigned long loop_count;

midi_cmd cmd;

uint8_t current_led;
uint8_t target_led;
unsigned long last_led_change;
uint8_t led_changed = 0;
uint8_t push_count;

struct in_led {
    unsigned long last_on;
    bool state;

#if defined(DEBUG_LED)
    unsigned long start_on;
    unsigned long pulses;
    unsigned long busy;

    enum {
        MAX_D = 10,
        MAX_P = 1000,
        MAX_PD = MAX_P / MAX_D
    };
    unsigned long pulses_d[MAX_D];
    unsigned long busy_d[MAX_D];
    unsigned long start_d;
#endif

    void setup() {
        last_on = 0;
        state = false;
    }

    void set_state(unsigned long t, bool s) {
#if defined(DEBUG_LED)
        if (state != s) {
            if (s) {
                ++pulses;
                start_on = t;

                if (t - start_d < MAX_P) {
                    ++pulses_d[ (t - start_d) / MAX_PD ];
                }
            } else {
                busy += (t - start_on);

                if (t - start_d < MAX_P) {
                    busy_d[ (t - start_d) / MAX_PD ] += (t - start_on);
                }
            }

            state = s;
        }
#else
        state = s;
#endif

        if (s) {
            last_on = t;
        }
    }

    operator bool() const {
        return state;
    }

#if defined(DEBUG_LED)
    void set_start_d(unsigned long t) {
        start_d = t;
        for (uint8_t i = 0; i < MAX_D; i++) {
            debug(5, "  ", pulses_d[i], ", ", busy_d[i]);
            pulses_d[i] = 0;
            busy_d[i] = 0;
        }
        debug(5, "  = ", pulses, ", ", busy, " ms");
        pulses = 0;
        busy = 0;
    }
#endif
};

in_led in[MAX_LED];


struct btn_timer_callback {
    void operator ()(unsigned long);
};

using btn_timer_type = artl::timer<btn_timer_callback>;

btn_timer_type btn_timer;

enum {
    BTN_RELEASE,
    BTN_PUSH,
    BTN_END
};

uint8_t btn_timer_state = BTN_END;

inline void process_midi_cmd(unsigned long t);
inline void btn_push(unsigned long t);

inline bool chk_current(unsigned long t, uint8_t n)
{
    if (!in[n] || n == current_led) {
        return false;
    }

    for (uint8_t i = 0; i < MAX_LED; i++) {
        if (i == n) { continue; }

        if (t - in[i].last_on < MIN_EXCLUSIVE_TIME) {
            return false;
        }
    }

    current_led = n;
    ++led_changed;
    last_led_change = t;

    debug(5, t, " led change ", current_led);

    return true;
}

volatile unsigned long led_changes = 0;

inline void on_led_change(unsigned long t) {
    led_changes++;

    in[0].set_state(t, led_in_0().read());
    in[1].set_state(t, led_in_1().read());
    in[2].set_state(t, led_in_2().read());
    in[3].set_state(t, led_in_3().read());

    for (uint8_t i = 0; i < MAX_LED; i++) {
        chk_current(t, i);
    }
}

ISR (PCINT1_vect) {
    midi_in.recv();
}

ISR (PCINT0_vect) {
    on_led_change(millis());
}

using ld = artl::digital_out<4>;
using clk = artl::digital_out<5>;
using qh = artl::digital_in<8>;

using piso = artl::piso<uint8_t, artl::lsb_first, qh, ld, clk>;

void setup()
{
    btn().input();
    led_in_0().setup();
    led_in_1().setup();
    led_in_2().setup();
    led_in_3().setup();

    loop_count = 0;

    midi_in.begin(31250);

    cmd.reset();

    target_led = 0;
    current_led = 0;
    last_led_change = 0;
    push_count = 0;

#if defined(DEBUG)
    debug_level_ = 5;

#if defined(DEBUG_TWI)
    debug_twi.setup();
#endif
#endif

    led_0_int().enable();
    led_1_int().enable();
    led_2_int().enable();
    led_3_int().enable();

    for (uint8_t i = 0; i < MAX_LED; i++) {
        in[i].setup();
    }

    piso().setup();
}

inline uint8_t
get_channel() {
    return piso().read() >> 4;
}

unsigned long last_print = 0;

void loop()
{
    unsigned long t = millis();
    loop_count++;

    while (!cmd && midi_in.available()) {
        cmd.read(midi_in.read());
    }

    if (cmd) {
        process_midi_cmd(t);
    }

    btn_timer.update(t);

#if defined(DEBUG) && defined(DEBUG_LOOP)
    if (t - last_print > 1000) {
        debug(5, t, " ", loop_count, " ", led_changes, " ", current_led, " ", get_channel());

#if defined(DEBUG_LED)
        in[0].set_start_d(t);
#endif

        last_print = t;
        loop_count = 0;

        cs();

        led_changes = 0;
    }
#endif

    artl::yield();
}

inline void
process_midi_cmd(unsigned long t)
{
    debug(4, t, " midi cmd: ", cmd.command(), " ", cmd.program());

    if (cmd.command() == midi_cmd::CMD_PROG_CHANGE &&
        cmd.channel() == get_channel()) {

        uint8_t new_target = cmd.program() % MAX_LED;

        if (btn_timer_state == BTN_END) {
            push_count = (push_count + new_target + MAX_LED - current_led) % MAX_LED;
        } else {
            push_count = (push_count + new_target + MAX_LED - target_led) % MAX_LED;
        }

        target_led = new_target;

        debug(5, t, " midi cmd: ", target_led, " ", push_count);

        if (push_count && btn_timer_state == BTN_END) {
            btn_push(t);
        }
    }

    cmd.reset();
}

inline void
schedule(unsigned long t, unsigned long dt, uint8_t state)
{
    debug(5, t, " schedule: ", dt, " ", state);

    btn_timer_state = state;

    btn_timer.schedule(t + dt);
}

inline void
btn_push(unsigned long t)
{
    --push_count;
    debug(4, t, " btn_push: ", push_count);

    led_changed = 0;

    btn().output();
    btn().low();

    schedule(t, PUSH_TIME, BTN_RELEASE);
}

inline void
btn_timer_callback::operator()(unsigned long t)
{
    debug(4, t, " on_timer: ", btn_timer_state, " ", push_count);

    switch (btn_timer_state) {
    case BTN_RELEASE:
        btn().input();

        schedule(t, PUSH_PAUSE, BTN_PUSH);
        break;

    case BTN_PUSH:
        if (push_count > 0) {
            btn_push(t);
        } else {
            btn_timer_state = BTN_END;
        }
        break;
    }
}
