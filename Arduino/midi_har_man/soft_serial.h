/*
soft_serial.h (formerly SoftwareSerial.h, formerly NewSoftSerial.h) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

#pragma once

#include <inttypes.h>
#include <Stream.h>

#include "artl/digital_in.h"
#include "artl/digital_out.h"
#include "artl/pin_change_int.h"

/******************************************************************************
* Definitions
******************************************************************************/

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

// Return num - sub, or 1 if the result would be < 1
template<long A, long B, bool GR = (A > B) >
struct subtract_cap
{
    enum { value = A - B };
};

template<long A, long B>
struct subtract_cap<A, B, false>
{
    enum { value = 1 };
};

template<uint8_t IN_PIN, uint8_t OUT_PIN, unsigned long SPEED, bool inverse_logic = false, int RX_BUF = 64>
class soft_serial : public Stream
{
public:
  enum {
      // Precalculate the various delays, in number of 4-cycle delays
      bit_delay = (F_CPU / SPEED) / 4,

      // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
      // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
      // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
      // These are all close enough to just use 15 cycles, since the inter-bit
      // timings are the most critical (deviations stack 8 times)
      tx_delay = subtract_cap<bit_delay, 15 / 4>::value,

      #if GCC_VERSION > 40800
      // Timings counted from gcc 4.8.2 output. This works up to 115200 on
      // 16Mhz and 57600 on 8Mhz.
      //
      // When the start bit occurs, there are 3 or 4 cycles before the
      // interrupt flag is set, 4 cycles before the PC is set to the right
      // interrupt vector address and the old PC is pushed on the stack,
      // and then 75 cycles of instructions (including the RJMP in the
      // ISR vector table) until the first delay. After the delay, there
      // are 17 more cycles until the pin value is read (excluding the
      // delay in the loop).
      // We want to have a total delay of 1.5 bit time. Inside the loop,
      // we already wait for 1 bit time - 23 cycles, so here we wait for
      // 0.5 bit time - (71 + 18 - 22) cycles.
      rx_delay_centering = subtract_cap<bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4>::value,

      // There are 23 cycles in each loop iteration (excluding the delay)
      rx_delay_intrabit = subtract_cap<bit_delay, 23 / 4>::value,

      // There are 37 cycles from the last bit read to the start of
      // stopbit delay and 11 cycles from the delay until the interrupt
      // mask is enabled again (which _must_ happen during the stopbit).
      // This delay aims at 3/4 of a bit time, meaning the end of the
      // delay will be at 1/4th of the stopbit. This allows some extra
      // time for ISR cleanup, which makes 115200 baud at 16Mhz work more
      // reliably
      rx_delay_stopbit = subtract_cap<bit_delay * 3 / 4, (37 + 11) / 4>::value,
      #else // Timings counted from gcc 4.3.2 output
      // Note that this code is a _lot_ slower, mostly due to bad register
      // allocation choices of gcc. This works up to 57600 on 16Mhz and
      // 38400 on 8Mhz.
      rx_delay_centering = subtract_cap<bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4>;
      rx_delay_intrabit = subtract_cap<bit_delay, 11 / 4>;
      rx_delay_stopbit = subtract_cap<bit_delay * 3 / 4, (44 + 17) / 4>;
      #endif

      receive_pin = IN_PIN,
  };

  using rx_pin = artl::digital_in<IN_PIN>;
  using rx_pin_int = artl::pin_change_int<IN_PIN>;
  using tx_pin = artl::digital_out<OUT_PIN>;

private:
  bool overflow_;

  // static data
  uint8_t buf_[RX_BUF]; 
  volatile uint8_t tail_;
  volatile uint8_t head_;

  // private static method for timing
  static inline void tunedDelay(uint16_t delay) {
    _delay_loop_2(delay);
  }

public:
  // public methods
  void begin(unsigned long);
  bool overflow() { bool ret = overflow_; if (ret) overflow_ = false; return ret; }
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available() { return (tail_ + RX_BUF - head_) % RX_BUF; }
  virtual void flush() { }
  operator bool() { return true; }

  using Print::write;

  inline void recv() __attribute__((__always_inline__));
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round


// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 6 // 11
#define _DEBUG_PIN2 5 // 13
// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <util/delay_basic.h>

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
#if _DDEBUG
inline void debug_pulse(uint8_t pin, uint8_t count)
{
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
}
#else
inline void debug_pulse(uint8_t, uint8_t) {}
#endif

#if _DEBUG
inline void debug_1_pulse() {
    using debug_1_pin = artl::digital_out<_DEBUG_PIN1>;

    debug_1_pin().high();
    debug_1_pin().low();
}

inline void debug_2_pulse() {
    using debug_2_pin = artl::digital_out<_DEBUG_PIN2>;

    debug_2_pin().high();
    debug_2_pin().low();
}
#else
inline void debug_1_pulse() { }
inline void debug_2_pulse() { }
#endif

//
// The receive routine called by the interrupt handler
//
template<uint8_t IN_PIN, uint8_t OUT_PIN, unsigned long SPEED, bool inverse_logic, int RX_BUF>
inline void soft_serial<IN_PIN, OUT_PIN, SPEED, inverse_logic, RX_BUF>::recv()
{

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (inverse_logic ? rx_pin().read() : !rx_pin().read())
  {
    // Disable further interrupts during reception, this prevents
    // triggering another interrupt directly after we return, which can
    // cause problems at higher baudrates.
    rx_pin_int().disable();

    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(rx_delay_centering);

    debug_pulse(_DEBUG_PIN2, 1);
    debug_1_pulse();

    // Read each of the 8 bits
    for (uint8_t i = 8; i > 0; --i)
    {
      tunedDelay(rx_delay_intrabit);
      d >>= 1;
      debug_pulse(_DEBUG_PIN2, 1);
      //debug_2_pulse();
      if (rx_pin().read_bit()) {
          d |= 0x80;
          debug_2_pulse();
      } else {
          debug_1_pulse();
      }
    }

    if (inverse_logic) {
      d = ~d;
    }

    // if buffer full, set the overflow flag and return
    uint8_t next = (tail_ + 1) % RX_BUF;
    if (next != head_)
    {
      // save new data in buffer: tail points to where byte goes
      buf_[tail_] = d; // save new byte
      tail_ = next;
    } 
    else 
    {
      debug_pulse(_DEBUG_PIN1, 1);
      debug_1_pulse();
      overflow_ = true;
    }

    // skip the stop bit
    tunedDelay(rx_delay_stopbit);
    debug_pulse(_DEBUG_PIN1, 1);
    debug_1_pulse();

    // Re-enable interrupts when we're sure to be inside the stop bit
    rx_pin_int().enable();
  }

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}

//
// Public methods
//

template<uint8_t IN_PIN, uint8_t OUT_PIN, unsigned long SPEED, bool inverse_logic, int RX_BUF>
void soft_serial<IN_PIN, OUT_PIN, SPEED, inverse_logic, RX_BUF>::begin(unsigned long)
{
  rx_pin().setup();
  tx_pin().setup();

  if (inverse_logic) {
    tx_pin().low();
  } else {
    tx_pin().high();
    rx_pin().pullup();
  }

  rx_pin_int().enable();

  overflow_ = false;
}

// Read data from buffer
template<uint8_t IN_PIN, uint8_t OUT_PIN, unsigned long SPEED, bool inverse_logic, int RX_BUF>
inline int soft_serial<IN_PIN, OUT_PIN, SPEED, inverse_logic, RX_BUF>::read()
{
  // Empty buffer?
  if (head_ == tail_) {
    return -1;
  }

  // Read from "head"
  uint8_t d = buf_[head_]; // grab next byte
  head_ = (head_ + 1) % RX_BUF;
  return d;
}

template<uint8_t IN_PIN, uint8_t OUT_PIN, unsigned long SPEED, bool inverse_logic, int RX_BUF>
size_t soft_serial<IN_PIN, OUT_PIN, SPEED, inverse_logic, RX_BUF>::write(uint8_t b)
{
  if (tx_delay == 0) {
    setWriteError();
    return 0;
  }

  if (inverse_logic) {
    b = ~b;
  }

  // disabling interrupts breaks receive MIDI commands
  // write now used only for debugging, so better we break the debug output
  // than MIDI command receive...

  // uint8_t oldSREG = SREG;
  // cli();  // turn off interrupts for a clean txmit

  if (inverse_logic) {
    tx_pin().high();
  } else {
    tx_pin().low();
  }

  tunedDelay(tx_delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i) {
    tx_pin().write(b & 1);

    tunedDelay(tx_delay);
    b >>= 1;
  }

  if (inverse_logic) {
    tx_pin().low();
  } else {
    tx_pin().high();
  }

  // SREG = oldSREG; // turn interrupts back on

  tunedDelay(tx_delay);

  return 1;
}

template<uint8_t IN_PIN, uint8_t OUT_PIN, unsigned long SPEED, bool inverse_logic, int RX_BUF>
int soft_serial<IN_PIN, OUT_PIN, SPEED, inverse_logic, RX_BUF>::peek()
{
  // Empty buffer?
  if (head_ == tail_) {
    return -1;
  }

  // Read from "head"
  return buf_[head_];
}
