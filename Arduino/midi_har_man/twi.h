
#pragma once

#include <stdint.h>

template<typename SDA_PIN, typename SCL_PIN>
struct twi : public Stream {
    using sda = SDA_PIN;
    using scl = SCL_PIN;

    using Print::write;

    void setup() {
        sda().output();
        scl().output();
    }

    void start(uint8_t b) {
        scl().high();

        sda().low();
        scl().low();

        write(b | 0x01);
    }

    virtual size_t write(uint8_t b) {
        // scl() expected to be low

        // Write each of the 8 bits
        for (uint8_t i = 8; i > 0; --i) {
            sda().write(b & 0x80);

            scl().high();

            scl().low();

            b <<= 1;
        }

        sda().low();
        // sda().input();

        scl().high();
        // sda().read();
        scl().low();

        return 1;
    }

    void end() {
        scl().high();
        sda().high();

        scl().low();
    }

    virtual int peek() { return 0; }

    virtual int read() { return 0; }
    virtual int available() { return 0; }
};
