#pragma once

#include "artl/digital_out.h"

struct led_array {
    using led_0 = artl::digital_out<10>;
    using led_1 = artl::digital_out<16>;
    using led_2 = artl::digital_out<14>;
    using led_3 = artl::digital_out<15>;

    enum {
        MAX_LED = 4
    };

    void setup() {
        led_0().setup();
        led_1().setup();
        led_2().setup();
        led_3().setup();

        TCCR0B = _BV(CS02);
    }

    operator uint8_t() const {
        return current_;
    }

    led_array& operator=(uint8_t l) {
        low();
        current_ = l % MAX_LED;
//        high();
        return *this;
    }

    led_array& operator ++() {
        return (*this) = current_ + 1;
    }

    void low() {
        switch (current_) {
          case 0:
              led_0().low();
              break;
          case 1:
              led_1().low();
              break;
          case 2:
              led_2().low();
              break;
          case 3:
              led_3().low();
              break;
        }
    }

    void high() {
        switch (current_) {
          case 0:
              led_0().high();
              break;
          case 1:
              led_1().high();
              break;
          case 2:
              led_2().high();
              break;
          case 3:
              led_3().high();
              break;
        }
    }

    uint8_t current_ = 0;
};


