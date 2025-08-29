module;

#include <stdio.h>
#include "hardware/pio.h"
#include "stepper.pio.h"

#include <array>

import pio_irq;

// this is to work around RISC compiler bug solved in 14.2
// https://gcc.gnu.org/pipermail/gcc-patches/2024-October/665109.html#:~:text=This%20patch%20leaves%20a%20couple,):%20...this.%20(
// TODO once fixed, replace TRANSLATION_BUG_INLINE with inline
#define TRANSLATION_BUG_INLINE


export module stepper;
export namespace stepper {

// implementations are in ./stepper_impl.cpp
// max steps taken is 2147483647 (highest number that fits in 31 bits)

/*  Stepper motor command wrapper: seteps to take, and direction
    lightweight 
*/
class command {
public:
    inline command(uint32_t steps, bool reverse) : cmd_(steps << 1 | (reverse ? 0 : 1)) {
        assert(steps <= (UINT32_MAX >> 1)); 
    }
    inline operator uint32_t() const { return cmd_; }
    friend uint32_t steps(const command &cmd);
    friend bool reverse(const command &cmd);
private:
    uint32_t cmd_;
};

inline uint32_t steps(const command &cmd) { return cmd.cmd_ >> 1; }
inline bool reverse(const command &cmd) { return !(cmd.cmd_ & 1); }

/*  Stepper motor wrapper for PIO state machine
    this class can be used as object,
    but if prefered, the static fuctions can be used without creating an object, 
    if developer preferes API style development.
*/
class stepper_controller {
public:
    stepper_controller(PIO pio, uint sm) : pio_(pio), sm_(sm) {}
    virtual ~stepper_controller() {}

    // API style methods, can be executed without creating an object
    // API style: write steps to TX FIFO. State machine will copy this into X
    static TRANSLATION_BUG_INLINE void take_steps(PIO pio, uint sm, const command& cmd) {
        pio_sm_put_blocking(pio, sm, cmd);
    }
    // API style: slow down motor
    // delay adds clock ticks to both halve periods of a pulse
    // call when the state machine is free. It interferes with activities
    static void set_delay(PIO pio, uint sm, uint32_t delay);
    // end API style methods

    // write steps to TX FIFO. State machine will copy this into X
    inline void take_steps(const command& cmd) { take_steps(pio_, sm_, cmd); }
    // slow down motor
    // delay adds clock ticks to both halve periods of a pulse
    // call when the state machine is free. It interferes with activities
    inline void set_delay(uint32_t delay) { set_delay(pio_, sm_, delay); }

    // state machine config
    TRANSLATION_BUG_INLINE void pio_init(uint dir_pin, float clock_divider) {
        stepper_program_init(pio_, sm_, pio_offset_[PIO_NUM(pio_)], dir_pin, clock_divider);
    }
    // enable or disable state machine
    TRANSLATION_BUG_INLINE void enable(bool enable) { pio_sm_set_enabled(pio_, sm_, enable); }
    // program the pio and store offset, to be called by user before a pio is used.
    static TRANSLATION_BUG_INLINE void pio_program(PIO pio) {
        pio_offset_[PIO_NUM(pio)] = pio_add_program(pio, &stepper_program);
    }
protected:
    PIO pio_;
    uint sm_;
    static uint pio_offset_[NUM_PIOS];
};

/*  Stepper motor for PIO state machine, 
    with interrupt and notification support:
    It can notify the caller that a command is finished,
    and / or it can report the number of commands executed
*/
class stepper_callback_controller : public stepper_controller {
using notifier_t = void (*)(const stepper_callback_controller&); // callback definition
private:

    /*
    PIO interrupts can only call functions without parameters. They can't call object members.
    This static embedded utility class helps matching interrupts to the relevant object.
    */
    class interrupt_manager {
    private:
        interrupt_manager() = delete;  // static class. prevent instantiating.
    public:        
        // if an object is currently handling a pio + sm combination, it will 
        // be replaced and will no longer receive interrupts
        // return false as warning if an existing combination is replaced
        static bool register_stepper(stepper_callback_controller * stepper, bool set);

        // PIO API doesn't accept a callback with parameters, so I can't pass the PIO instance
        // provide a parameter-less method for ezach PIO is a reasonable solution
        // only task is to call interrupt_handler() and passing it the PIO indicated in the name.
        // Without overhead: optimised out inrelease code
        static inline void interrupt_handler_PIO0() { interrupt_handler(pio0); }    
        static inline void interrupt_handler_PIO1() { interrupt_handler(pio1); }
#if (NUM_PIOS > 2) // pico 2       
        static inline void interrupt_handler_PIO2() { interrupt_handler(pio2); }
#endif        
    private:
        // forwards the interrupt to the surrounding class
        static void interrupt_handler(PIO pio);
        // utility calculates the index for the object staht serves a state machine in steppers_
        static inline size_t index_for(PIO pio, uint sm) { return PIO_NUM(pio) * 4 + sm; }
        // keep pointer of objects that serve the state machines
        // 2-D array with slot for all possible state machines: PIO0[0..3], PIO1[0..3], ...
        static std::array<stepper_callback_controller *, NUM_PIOS * 4> steppers_;
    };   
public:
    stepper_callback_controller(PIO pio, uint sm) : stepper_controller(pio,sm), commands_(0U),
        callback_(nullptr) { interrupt_manager::register_stepper(this, true); }
    virtual ~stepper_callback_controller() { interrupt_manager::register_stepper(this, false); }

    // return commands completed
    inline uint commands() const { return commands_; }
    // reset commands completed to 0
    inline void reset_commands() { commands_ = 0U; }

    // register object as interrupt service for its state machine
    void register_pio_interrupt(uint irq_channel, bool enable);
    // user code class to call when a command is finished.
    // Pass immutable reference to object as user info
    inline void on_complete_callback(notifier_t callback) { callback_ = callback; }
private:
    void handler();
    volatile uint commands_; // volatile: updated by interrupt handler
    notifier_t callback_;
};

// relative interrupt handler
// the PIO interrupt manager
using pio_irq_manager_t = pio_irq::pio_irq<stepper_callback_controller, stepper_PIO_IRQ_DONE>;

} // namespace stepper