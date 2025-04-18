module;

#include "hardware/pio.h"
#include <utility>

export module pio_irq_util;
export namespace pio_irq_util {

    // sm   irq int MOD SFTL    sm      irq     int     MOD     SFTL
    // 0    0   1   0   1       0000    0000    0001    0000    0001
    // 0    1   2   1   2       0000    0001    0010    0001    0010
    // 0    2   4   2   4       0000    0010    0100    0010    0100
    // 0    3   8   3   8       0000    0011    1000    0011    1000
    // 1    0   2   1   2       0001    0000    0010    0001    0010
    // 1    1   4   2   4       0001    0001    0100    0010    0100
    // 1    2   8   3   8       0001    0010    1000    0011    1000
    // 1    3   1   0   1       0001    0011    0001    0000    0001
    // 2    0   4   2   4       0010    0000    0100    0010    0100
    // 2    1   8   3   8       0010    0001    1000    0011    1000
    // 2    2   1   0   1       0010    0010    0001    0000    0001
    // 2    3   2   1   2       0010    0011    0010    0001    0010
    // 3    0   8   3   8       0011    0000    1000    0011    1000
    // 3    1   1   0   1       0011    0001    0001    0000    0001
    // 3    2   2   1   2       0011    0010    0010    0001    0010
    // 3    3   4   2   4       0011    0011    0100    0010    0100
    
// 10 (REL): the state machine ID (0…3) is added to the IRQ flag index, by way of
// modulo-4 addition on the two LSBs
inline uint relative_interrupt(const uint32_t ir, const uint sm) {
    // TODO validate for Pico 2 interrupts 4 .. 7
    uint32_t retval = ir & 0x03; // last 2 bits
    retval += sm; // add relative value (is sm)
    retval = retval % 4; // mod 4
    // TODO most likely I have to restore bits 31..2 here to work with Pico 2
    // but I don't have one to test it
    retval |= ir & 0xfffffffc;
    return retval;
}

inline pio_interrupt_source interrupt_source(const pio_interrupt_source is, const uint32_t ir) {
    return static_cast<pio_interrupt_source>(std::to_underlying(is) + ir);
}

uint sm_from_interrupt(const uint32_t irq_val, const uint32_t ir) {
    // TODO validate for Pico 2 interrupts 4 .. 7
    uint i;
    for (i = 0; i < 4; i++) { // should be sm 0 .. 3
        if (irq_val == 1 << relative_interrupt(ir, i)) {
            break;
        }
    }
    assert(i != 4); // develop check there has to be a bit
    return i;
}

} // namespace pio_irq_util