module;

#include "stepper.pio.h"

#include <array>

import pio_irq;

module stepper;

namespace stepper {


void stepper_controller::set_delay(PIO pio, uint sm, uint32_t delay) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put(pio, sm, delay);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

// relative interrupt handler
using pio_irq_manager_t = pio_irq::pio_irq<stepper_callback_controller, stepper_PIO_IRQ_DONE>;

stepper_callback_controller::stepper_callback_controller(PIO pio, uint sm) : 
        stepper_controller(pio,sm), commands_(0U), callback_(nullptr) { 
    pio_irq_manager_t::register_handler(pio_, sm_, this, true); 
}

stepper_callback_controller::~stepper_callback_controller() { 
    pio_irq_manager_t::register_handler(pio_, sm_, this, false); 
}

void stepper_callback_controller::register_pio_interrupt(uint irq_channel, bool enable) {
    pio_irq_manager_t::register_interrupt(irq_channel, pio_, sm_, enable);
}

// initialise static class data
// static pio program offset
// uint stepper_controller::pio_offset_[NUM_PIOS];

} // namespace stepper