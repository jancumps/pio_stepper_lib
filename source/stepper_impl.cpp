module;

#include "stepper.pio.h"

#include <array>

import pio_irq_util;

module stepper;

namespace stepper {

void stepper_controller::set_delay(PIO pio, uint sm, uint32_t delay) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put(pio, sm, delay);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

bool stepper_callback_controller::interrupt_manager::register_stepper(stepper_callback_controller * stepper, bool set) {
    size_t idx = index_for(stepper->pio_, stepper->sm_);
    stepper_callback_controller *old = steppers_[idx];
    steppers_[idx] = set ? stepper : nullptr;
    return set ? old == nullptr : true;
}

void stepper_callback_controller::interrupt_manager::interrupt_handler(PIO pio) {
    uint sm = pio_irq_util::sm_from_interrupt(pio->irq, stepper_PIO_IRQ_DONE);
    stepper_callback_controller *stepper =  steppers_[index_for(pio, sm)];
    if (stepper != nullptr) {
        stepper -> handler();
    }
}

void stepper_callback_controller::register_pio_interrupt(uint irq_channel, bool enable) {
    assert (irq_channel < 2); // develop check that we use 0 or 1 only
    uint irq_num = PIO0_IRQ_0 + 2 * PIO_NUM(pio_) + irq_channel;
    irq_handler_t handler = nullptr;

    if (irq_channel == 0) {
        pio_set_irq0_source_enabled(pio_, pio_irq_util::interrupt_source(pis_interrupt0, 
            pio_irq_util::relative_interrupt(stepper_PIO_IRQ_DONE, sm_)), true);
    } else {
        pio_set_irq1_source_enabled(pio_, pio_irq_util::interrupt_source(pis_interrupt0, 
            pio_irq_util::relative_interrupt(stepper_PIO_IRQ_DONE, sm_)), true);
    }

    switch (PIO_NUM(pio_)) {
    case 0:
        handler = interrupt_manager::interrupt_handler_PIO0;
        break;
    case 1:
        handler = interrupt_manager::interrupt_handler_PIO1;
        break;
#if (NUM_PIOS > 2) // pico 2       
    case 2:
        handler = stepper_interrupt_manager::interrupt_handler_PIO2;
        break;
#endif            
    }

    irq_set_exclusive_handler(irq_num, handler);  //Set the handler in the NVIC
    if (enable) {
        irq_set_enabled(irq_num, true);
    }
}

void stepper_callback_controller::handler() {
    uint ir = pio_irq_util::relative_interrupt(stepper_PIO_IRQ_DONE, sm_);
    assert(pio_->irq == 1 << ir); // develop check: interrupt is from the correct state machine
    commands_ = commands_ + 1;
    pio_interrupt_clear(pio_, ir);
    if (callback_ != nullptr) {
        (callback_)( *this);
    }
}

// initialise static class data
// static pio program offset
uint stepper_controller::pio_offset_[NUM_PIOS];
// static data member must be initialised outside of the class, or the linker will not capture it
std::array<stepper_callback_controller *, NUM_PIOS * 4> stepper_callback_controller::interrupt_manager::steppers_;

} // namespace stepper