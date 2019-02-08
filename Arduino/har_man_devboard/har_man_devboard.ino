
#include "artl/digital_pin.h"
#include "artl/digital_out.h"
#include "artl/digital_in.h"

#include "artl/button.h"

#include "artl/tc.h"

#include "led_array.h"

#include <avr/interrupt.h>
#include <avr/sleep.h>

using my_btn_in = artl::digital_in<A0>;

// Configure SPI clock (in Hz).
// E.g. for an ATtiny @ 128 kHz: the datasheet states that both the high and low
// SPI clock pulse must be > 2 CPU cycles, so take 3 cycles i.e. divide target
// f_cpu by 6:
//     #define SPI_CLOCK            (128000/6)
//
// A clock slow enough for an ATtiny85 @ 1 MHz, is a reasonable default:

#define SPI_CLOCK 		(1000000/6)

#define RESET     6 // Use pin to reset the target rather than SS
#define LED_HB    5
#define LED_ERR   3
#define LED_PMODE 4

// Uncomment following line to use the old Uno style wiring
// (using pin 11, 12 and 13 instead of the SPI header) on Leonardo, Due...

#define PIN_MOSI  8
#define PIN_MISO  9
#define PIN_SCK   7


using led_hb = artl::digital_out<LED_HB>;
using led_err = artl::digital_out<LED_ERR>;
using led_pmode = artl::digital_out<LED_PMODE>;

using pin_reset = artl::digital_pin<RESET>;
using pin_mosi = artl::digital_pin<PIN_MOSI>;
using pin_miso = artl::digital_pin<PIN_MISO>;
using pin_sck = artl::digital_pin<PIN_SCK>;

// Configure the serial port to use.
//
// Prefer the USB virtual serial port (aka. native USB port), if the Arduino has one:
//   - it does not autoreset (except for the magic baud rate of 1200).
//   - it is more reliable because of USB handshaking.
//
// Leonardo and similar have an USB virtual serial port: 'Serial'.
// Due and Zero have an USB virtual serial port: 'SerialUSB'.
//
// On the Due and Zero, 'Serial' can be used too, provided you disable autoreset.
// To use 'Serial': #define Serial Serial

// Configure the baud rate:

#define BAUDRATE	19200
// #define BAUDRATE	115200
// #define BAUDRATE	1000000


#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
static const uint8_t STK_OK      = 0x10;
static const uint8_t STK_FAILED  = 0x11;
static const uint8_t STK_UNKNOWN = 0x12;
static const uint8_t STK_INSYNC  = 0x14;
static const uint8_t STK_NOSYNC  = 0x15;
static const uint8_t CRC_EOP     = 0x20; //ok it is a space...

void pulse(int pin, int times);

#define SPI_MODE0 0x00

class SPISettings {
public:
    // clock is in Hz
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) : clock(clock) {
        (void) bitOrder;
        (void) dataMode;
    };

private:
    uint32_t clock;

    friend class BitBangedSPI;
};

class BitBangedSPI {
public:
    void begin() {
        pin_sck().low();
        pin_mosi().low();

        pin_sck().output();
        pin_mosi().output();
        pin_miso().input();
    }

    void beginTransaction(SPISettings settings) {
        pulseWidth = (500000 + settings.clock - 1) / settings.clock;
        if (pulseWidth == 0) {
            pulseWidth = 1;
        }
    }

    void end() {}

    uint8_t transfer (uint8_t b) {
        for (unsigned int i = 0; i < 8; ++i) {
            pin_mosi().write(b & 0x80);
            pin_sck().high();

            delayMicroseconds(pulseWidth);
            b = (b << 1) | (pin_miso().read() ? 1 : 0);

            pin_sck().low(); // slow pulse
            delayMicroseconds(pulseWidth);
        }
        return b;
    }

  private:
    unsigned long pulseWidth; // in microseconds
};

static BitBangedSPI SPI;

led_array my_leds;

struct my_btn_handler : public artl::default_button_handler {
    void on_down(uint8_t down, unsigned long) {
        if (!down) return;

        ++my_leds;
    }
};

artl::button< my_btn_handler > my_btn;

using tc = artl::tc<4>;

enum {
    ON_CNT = 130,
    MAX_CNT = 156
};

unsigned long blink_cnt = 0;
uint8_t noise_id = 0;

enum {
    MAX_NOISE = 100,
    OFF_NOISE = 1
};

using led_noise = led_array::led_2;

ISR (TIMER4_COMPA_vect) {
    if (tc().cnt() >= MAX_CNT) {
        tc().cnt() = 0;
        tc().ocra() = ON_CNT;
        my_leds.low();

        if (noise_id < OFF_NOISE) {
            led_noise().low();
        }

        noise_id = (noise_id + 1) % MAX_NOISE;

        return;
    }

    if (tc().cnt() >= ON_CNT) {
        my_leds.high();
        tc().ocra() = MAX_CNT;
        blink_cnt++;

        if (noise_id < OFF_NOISE) {
            led_noise().high();
        }
    }
}

inline void
yield()
{
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_cpu();
}

void setup() {
    Serial.begin(BAUDRATE);
    Serial1.begin(31250);

    led_pmode().output();
    pulse(led_pmode(), 2);

    led_err().output();
    pulse(led_err(), 2);

    led_hb().output();
    pulse(led_hb(), 2);

    my_btn_in().setup();
    my_btn_in().pullup();

    my_leds = 0;

    tc().ocra() = ON_CNT;
    tc().setup(0, 0, 0, tc::cs::presc_2048);
    tc().oca().enable();
}

int error = 0;
bool pmode = false;
// address for reading and writing, set by 'U' command
unsigned int here;
uint8_t buff[256]; // global block storage

unsigned long loop_cnt = 0;

void inc_error() {
    error++;
    led_err().high();
}

#define beget16(addr) (*addr * 256 + *(addr+1) )
struct parameter {
    uint8_t devicecode;
    uint8_t revision;
    uint8_t progtype;
    uint8_t parmode;
    uint8_t polling;
    uint8_t selftimed;
    uint8_t lockbytes;
    uint8_t fusebytes;
    uint8_t flashpoll;
    uint16_t eeprompoll;
    uint16_t pagesize;
    uint16_t eepromsize;
    uint32_t flashsize;
};

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
    static unsigned long last_time = 0;
    unsigned long now = millis();

    if ((now - last_time) < 40) {
        return;
    }

    last_time = now;
    if (hbval > 192) hbdelta = -hbdelta;
    if (hbval < 32) hbdelta = -hbdelta;
    hbval += hbdelta;
    analogWrite(led_hb(), hbval);
}

static bool rst_active_high;

void reset_target(bool reset) {
    pin_reset().write((reset && rst_active_high) || (!reset && !rst_active_high));
}

unsigned long last_t = 0;

void loop(void) {
    loop_cnt++;

    // light the heartbeat LED
    heartbeat();

    if (Serial.available()) {
        avrisp();
    }

    if (!pmode) {
        while (Serial1.available()) {
            uint8_t n = Serial1.read();
            Serial.write(n);
        }
    }

    unsigned long t = millis();

    my_btn.update(!my_btn_in().read(), t);
/*
    if (t - last_t > 1000) {
        Serial.print( (t - last_t) );
        Serial.print(" ");
        Serial.print(loop_cnt);
        Serial.print(" ");
        Serial.print(blink_cnt);
        Serial.print(" ");
        Serial.println(" ");

        loop_cnt = 0;
        blink_cnt = 0;
        last_t = t;
    }
*/
    yield();
}

uint8_t getch() {
  while (!Serial.available());
  return Serial.read();
}
void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
  }
}

#define PTIME 30
void pulse(int pin, int times) {
    do {
        digitalWrite(pin, HIGH);
        delay(PTIME);
        digitalWrite(pin, LOW);
        delay(PTIME);
    } while (times--);
}

#define PROG_FLICKER 1

void prog_lamp(int state) {
#if (PROG_FLICKER)
    led_pmode().write(state);
#endif
}


uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    return SPI.transfer(d);
}

void empty_reply() {
    if (CRC_EOP == getch()) {
        Serial.write(STK_INSYNC);
        Serial.write(STK_OK);
    } else {
        inc_error();
        Serial.write(STK_NOSYNC);
    }
}

void breply(uint8_t b) {
    if (CRC_EOP == getch()) {
        Serial.write(STK_INSYNC);
        Serial.write(b);
        Serial.write(STK_OK);
    } else {
        inc_error();
        Serial.write(STK_NOSYNC);
    }
}

void get_version(uint8_t c) {
    switch (c) {
    case 0x80:
        breply(HWVER);
        break;
    case 0x81:
        breply(SWMAJ);
        break;
    case 0x82:
        breply(SWMIN);
        break;
    case 0x93:
        breply('S'); // serial programmer
        break;
    default:
        breply(0);
    }
}

void set_parameters() {
    // call this after reading parameter packet into buff[]
    param.devicecode = buff[0];
    param.revision   = buff[1];
    param.progtype   = buff[2];
    param.parmode    = buff[3];
    param.polling    = buff[4];
    param.selftimed  = buff[5];
    param.lockbytes  = buff[6];
    param.fusebytes  = buff[7];
    param.flashpoll  = buff[8];
    // ignore buff[9] (= buff[8])
    // following are 16 bits (big endian)
    param.eeprompoll = beget16(&buff[10]);
    param.pagesize   = beget16(&buff[12]);
    param.eepromsize = beget16(&buff[14]);

    // 32 bits flashsize (big endian)
    param.flashsize = buff[16] * 0x01000000
                      + buff[17] * 0x00010000
                      + buff[18] * 0x00000100
                      + buff[19];

    // AVR devices have active low reset, AT89Sx are active high
    rst_active_high = (param.devicecode >= 0xe0);
}

void start_pmode() {

    // Reset target before driving PIN_SCK or PIN_MOSI

    // SPI.begin() will configure SS as output, so SPI master mode is selected.
    // We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
    // So we have to configure RESET as output here,
    // (reset_target() first sets the correct level)
    reset_target(true);
    pin_reset().output();

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));

    // See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

    // Pulse RESET after PIN_SCK is low:
    pin_sck().low();
    delay(20); // discharge PIN_SCK, value arbitrarily chosen
    reset_target(false);
    // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
    // speeds above 20 KHz
    delayMicroseconds(100);
    reset_target(true);

    // Send the enable programming command:
    delay(50); // datasheet: must be > 20 msec
    spi_transaction(0xAC, 0x53, 0x00, 0x00);

    pmode = true;
    led_pmode().high();
}

void end_pmode() {
    SPI.end();

    // We're about to take the target out of reset so configure SPI pins as input
    pin_mosi().input();
    pin_sck().input();
    reset_target(false);

    pin_reset().input();

    pmode = false;
    led_pmode().low();
}

void universal() {
    uint8_t ch;

    fill(4);
    ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
    breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data) {
    spi_transaction(0x40 + 8 * hilo,
                    addr >> 8 & 0xFF,
                    addr & 0xFF,
                    data);
}

void commit(unsigned int addr) {
    prog_lamp(LOW);

    spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);

    if (PROG_FLICKER) {
        delay(PTIME);
        prog_lamp(HIGH);
    }
}

unsigned int current_page() {
    if (param.pagesize == 32) {
        return here & 0xFFFFFFF0;
    }
    if (param.pagesize == 64) {
        return here & 0xFFFFFFE0;
    }
    if (param.pagesize == 128) {
        return here & 0xFFFFFFC0;
    }
    if (param.pagesize == 256) {
        return here & 0xFFFFFF80;
    }
    return here;
}

void write_flash(int length) {
    fill(length);
    if (CRC_EOP == getch()) {
        Serial.write(STK_INSYNC);
        Serial.write(write_flash_pages(length));
    } else {
        inc_error();
        Serial.write(STK_NOSYNC);
    }
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  unsigned int page = current_page();
  while (x < length) {
    if (page != current_page()) {
      commit(page);
      page = current_page();
    }
    flash(LOW, here, buff[x++]);
    flash(HIGH, here, buff[x++]);
    here++;
  }

  commit(page);

  return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(unsigned int length) {
    // here is a word address, get the byte address
    unsigned int start = here * 2;
    unsigned int remaining = length;

    if (length > param.eepromsize) {
        inc_error();
        return STK_FAILED;
    }

    while (remaining > EECHUNK) {
        write_eeprom_chunk(start, EECHUNK);
        start += EECHUNK;
        remaining -= EECHUNK;
    }

    write_eeprom_chunk(start, remaining);
    return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length) {
  // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
  fill(length);
  prog_lamp(LOW);
  for (unsigned int x = 0; x < length; x++) {
    unsigned int addr = start + x;
    spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
    delay(45);
  }
  prog_lamp(HIGH);
  return STK_OK;
}

void program_page() {
    uint8_t result = STK_FAILED;
    unsigned int length = 256 * getch();

    length += getch();
    char memtype = getch();
    // flash memory @here, (length) bytes
    if (memtype == 'F') {
        write_flash(length);
        return;
    }

    if (memtype == 'E') {
        result = write_eeprom(length);
        if (CRC_EOP == getch()) {
            Serial.write(STK_INSYNC);
            Serial.write(result);
        } else {
            inc_error();
            Serial.write(STK_NOSYNC);
        }
        return;
    }
    Serial.write(STK_FAILED);
}

uint8_t flash_read(uint8_t hilo, unsigned int addr) {
  return spi_transaction(0x20 + hilo * 8,
                         (addr >> 8) & 0xFF,
                         addr & 0xFF,
                         0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x += 2) {
    uint8_t low = flash_read(LOW, here);
    Serial.write(low);
    uint8_t high = flash_read(HIGH, here);
    Serial.write(high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    Serial.write(ee);
  }
  return STK_OK;
}

void read_page() {
    char result = (char)STK_FAILED;
    int length = 256 * getch();
    length += getch();
    char memtype = getch();

    if (CRC_EOP != getch()) {
        inc_error();
        Serial.write(STK_NOSYNC);
        return;
    }

    Serial.write(STK_INSYNC);
    if (memtype == 'F') result = flash_read_page(length);
    if (memtype == 'E') result = eeprom_read_page(length);
    Serial.print(result);
}

void read_signature() {
    if (CRC_EOP != getch()) {
        inc_error();
        Serial.write(STK_NOSYNC);
        return;
    }
    Serial.write(STK_INSYNC);
    uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
    Serial.write(high);
    uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
    Serial.write(middle);
    uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
    Serial.write(low);
    Serial.write(STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////


////////////////////////////////////
////////////////////////////////////
void avrisp() {
    uint8_t ch = getch();

    if (!pmode) {
        switch (ch) {
        case 'v':
        case 'w':
        case 'x':
        case 'y':
        case 'z':
            Serial1.write( (uint8_t) 0xC2U);
            Serial1.write( (uint8_t) (ch - 'v' + 118) );
            return;
        }
    }

    switch (ch) {
    case '0': // signon
        error = 0;
        led_err().low();
        empty_reply();
        break;
    case '1':
        if (getch() == CRC_EOP) {
            Serial.write(STK_INSYNC);
            Serial.print("AVR ISP");
            Serial.write(STK_OK);
        } else {
            inc_error();
            Serial.write(STK_NOSYNC);
        }
        break;
    case 'A':
        get_version(getch());
        break;
    case 'B':
        fill(20);
        set_parameters();
        empty_reply();
        break;
    case 'E': // extended parameters - ignore for now
        fill(5);
        empty_reply();
        break;
    case 'P':
        if (!pmode) {
            start_pmode();
        }
        empty_reply();
        break;
    case 'U': // set address (word)
        here = getch();
        here += 256 * getch();
        empty_reply();
        break;
    case 0x60: //STK_PROG_FLASH
        getch(); // low addr
        getch(); // high addr
        empty_reply();
        break;

    case 0x61: //STK_PROG_DATA
        getch(); // data
        empty_reply();
        break;

    case 0x64: //STK_PROG_PAGE
        program_page();
        break;

    case 0x74: //STK_READ_PAGE 't'
        read_page();
        break;

    case 'V': //0x56
        universal();
        break;
    case 'Q': //0x51
        error = 0;
        led_err().low();
        end_pmode();
        empty_reply();
        break;

    case 0x75: //STK_READ_SIGN 'u'
        read_signature();
        break;

    // expecting a command, not CRC_EOP
    // this is how we can get back in sync
    case CRC_EOP:
        inc_error();
        Serial.write(STK_NOSYNC);
        break;

    // anything else we will return STK_UNKNOWN
    default:
        inc_error();
        if (CRC_EOP == getch()) {
           Serial.write(STK_UNKNOWN);
        } else {
           Serial.write(STK_NOSYNC);
        }
    }
}
