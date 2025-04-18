# pio_stepper_lib
Raspberry PIO autonomous stepper motor driver  
C++ library to run stepper motors in PIO state machines

- 4 motors can be controlled per PIO
- can be used with any stepper motor driver that supports PIN and DIR operation
- can handle as many commands as PIO FIFO accepts without waiting (default 8).
- each command can autonomously handle 2147483647 steps, and the spin direction
- can notify the calling program when an instruction is finished
- can report how many commands it processed

Example motor commands batch of 7 instructions:  
```
#include "hardware/pio.h"
import stepper; // PIO stepper lib

const uint dir = 4U; // implies that step is gpio 5
// config what PIO, SM and IRQ channel to use
const auto piostep = pio1;
const uint pio_irq = 0;
const uint sm = 2U;

const float clock_divider = 3; // works well for 8 microsteps

using motor_t = stepper::stepper_callback_controller;
motor_t motor1(piostep, sm);

void init_pio() {
    // program the pio used for the motors
    // do this only once per used pio
    motor_t::pio_program(piostep);

    // initialise and enable the motor state machine
    motor1.register_pio_interrupt(pio_irq, true);
    motor1.pio_init(dir, clock_divider);
    motor1.enable(true);
}

void on_complete(motor_t &stepper) {
    if (&motor1 == &stepper) {
        printf("motor1 executed command %d\n", motor1.commands());
    }
}

int main() {
    init_pio();

    std::array<stepper::command, 7> cmd{{
        {20 * microstep_x, true}, 
        {20 * microstep_x, false},
        {20 * microstep_x, false},
        {40 * microstep_x, false},
        {25 * microstep_x, true},
        {35 * microstep_x, true},
        {200 * microstep_x, true}}
    };

    motor1.on_complete_callback(on_complete); 

    motor1.set_delay(4300); // this is fast
    for(auto c : cmd) {
        motor1.take_steps(c);
    }
    while(motor1.commands() < cmd.size() ) {}

    return 0;
}


```