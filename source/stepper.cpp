module;

#include <stdio.h>
#include "hardware/pio.h"
#include "stepper.pio.h"

#include <array>

export module stepper;
export namespace stepper {

// implementations are in ./stepper_impl.cpp
// max steps taken is 2147483647 (highest number that fits in 31 bits)

/*  Stepper motor command wrapper
    lightweight 
*/
class command {
public:
    inline command(uint32_t steps, bool reverse) : cmd_(steps << 1 | (reverse ? 0 : 1)) {
        assert(steps <= (UINT32_MAX >> 1)); 
    }
    inline operator uint32_t() const { return cmd_; }
private:
    uint32_t cmd_;
};

/*  Stepper motor wrapper for PIO state machine
    this class can be used as object,
    but if prefered, the static fuctions can be used without creating an object, 
    if developer preferes API style development.
*/
class stepper_controller {
public:
    stepper_controller(PIO pio, uint sm) : pio_(pio), sm_(sm) {}
    virtual ~stepper_controller() {}
    // Write `steps` to TX FIFO. State machine will copy this into X
    static inline void take_steps(PIO pio, uint sm, const command& cmd) {
        pio_sm_put_blocking(pio, sm, cmd);
    }
    static inline void take_steps(PIO pio, uint sm, uint32_t steps, bool reverse) {
        take_steps(pio, sm, command(steps, reverse));
    }
    // call when the state machine is free. It interferes with activities
    static void set_delay(PIO pio, uint sm, uint32_t delay);
    inline void take_steps(const command& cmd) { take_steps(pio_, sm_, cmd); }
    inline void set_delay(uint32_t delay) { set_delay(pio_, sm_, delay); }
    // state machine config
    inline void pio_init(uint dir_pin, float clock_divider) {
        stepper_program_init(pio_, sm_, pio_offset_[PIO_NUM(pio_)], dir_pin, clock_divider);
    }
    // enable or disable state machine
    inline void enable(bool enable) { pio_sm_set_enabled(pio_, sm_, enable); }
    // program the pio and store offset, to be called by user before a pio is used.
    static inline void pio_program(PIO pio) {
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
using notifier_t = void (*)(stepper_callback_controller&); // callback definition
private:
    /*
    PIO interrupts can't call object members, 
    this embedded class helps translating interrupts to the relevant object
    also enforces this restriction: one stepper_interrupt object per state machine
    because this no longer a wrapper. We maintain state
    */
    class interrupt_manager {
    private:
        interrupt_manager() = delete;  // static classe. prevent instantiating.
    public:        
        // if an object is currently handling a pio + sm combination, it will 
        // be replaced and will no longer receive interrupts
        // return false if an existing combination is replaced
        static bool register_stepper(stepper_callback_controller * stepper, bool set);

        // PIO API doesn't accept a callback with parameters, so I can't pass the PIO instance
        // this is a reasonable solution without overhead
        static inline void interrupt_handler_PIO0() { interrupt_handler(pio0); }    
        static inline void interrupt_handler_PIO1() { interrupt_handler(pio1); }
#if (NUM_PIOS > 2) // pico 2       
        static inline void interrupt_handler_PIO2() { interrupt_handler(pio2); }
#endif        
    private:
        static void interrupt_handler(PIO pio);
        static inline size_t index_for(PIO pio, uint sm) { return PIO_NUM(pio) * 4 + sm; }
        // keep pointer to all possible objects
        static std::array<stepper_callback_controller *, NUM_PIOS * 4> steppers_;
    };   
public:
    stepper_callback_controller(PIO pio, uint sm) : stepper_controller(pio,sm), commands_(0U),
        callback_(nullptr) { interrupt_manager::register_stepper(this, true); }
    virtual ~stepper_callback_controller() { interrupt_manager::register_stepper(this, false); }
    inline uint commands() const { return commands_; }
    inline void reset_commands() { commands_ = 0U; }
    void register_pio_interrupt(uint irq_channel, bool enable);
    inline void on_complete_callback(notifier_t callback) { callback_ = callback; }
private:
    void handler();
    volatile uint commands_; // updated by interrupt handler
    notifier_t callback_;
};

} // namespace stepper