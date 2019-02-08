#pragma once

struct cs {
    uint8_t old_sreg;

    cs() : old_sreg(SREG) { cli(); }
    ~cs() { SREG = old_sreg; }
};
