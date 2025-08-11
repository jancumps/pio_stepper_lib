# pio_stepper_lib
Raspberry PIO autonomous stepper motor driver  
C++ library to run stepper motors in PIO state machines

- 4 motors can be controlled per PIO
- can be used with any stepper motor driver that supports STEP and DIR operation
- can handle as many commands as PIO FIFO accepts without waiting (default 8).
- each command can autonomously handle 2147483647 steps, and the spin direction
- can notify the calling program when a command has finished
- can report how many commands it processed

## documentation:
[1: intro, set up project and simple example](https://community.element14.com/products/raspberry-pi/b/blog/posts/raspberry-pio-stepper-library-documentation---1-intro-set-up-project-and-simple-example)  
[2: advanced example with notification](https://community.element14.com/products/raspberry-pi/b/blog/posts/raspberry-pio-stepper-library-documentation---2-advanced-example-with-notification)  
[3: control multiple motors with 1 or more PIOs](https://community.element14.com/products/raspberry-pi/b/blog/posts/raspberry-pio-stepper-library-documentation---3-control-multiple-motors-with-1-or-more-pios)  
[4: understand the step frequency](https://community.element14.com/products/raspberry-pi/b/blog/posts/raspberry-pio-stepper-library-documentation---4-understand-the-step-frequency)  
[5: simple ramp up and down](https://community.element14.com/products/raspberry-pi/b/blog/posts/raspberry-pio-stepper-library-documentation---5-simple-ramp-up-and-down)  

## video: 4 motors running on 1 PIO. Consecutive and Concurrent
<a href="http://www.youtube.com/watch?feature=player_embedded&v=Hhug2iatza0" target="_blank">
 <img src="http://img.youtube.com/vi/Hhug2iatza0/mqdefault.jpg" alt="Watch the video"/>
</a>

## Example: motor runs a batch of 7 commands:  
```
#include "hardware/pio.h"
import stepper; // PIO stepper lib

// pins
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
        {20, true}, 
        {20, false},
        {20, false},
        {40, false},
        {25, true},
        {35, true},
        {200, true}}
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

## Adding the lib to your Raspberry Pico project
The repository has its own makefile. you add it by fetching from it in your ```CMakeFiles.txt```. It is then available to your code as library stepper.
```
# your CMakeFiles.txt
# ... 

include(FetchContent)
FetchContent_Declare(stepper
  GIT_REPOSITORY "https://github.com/jancumps/pio_stepper_lib.git"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(stepper)

# ...

# add stepper as a library to your executable
add_executable(your_stepper_project)
# ...
target_link_libraries(your_stepper_project
        pico_stdlib
        hardware_gpio
        stepper
)
```
That is all it takes to integrate this design in your project.

## Other project considerations  
On a RP2350 (Pico2), the SDK default setting for ```PICO_MAX_SHARED_IRQ_HANDLERS``` limits the number of interrupt/callback enabled motors you can use. When you have more than 4 interrupt/callback enabled motors in your project, override the SDK setting in your ```CMakeFiles.txt``` with the number of motors in your code. Add it before Pico SDK import.
```
add_compile_definitions(PICO_MAX_SHARED_IRQ_HANDLERS=10)
```


## demo project that uses this lib: 
[Stepper Motor Control with Raspberry Pico PIO and DRV8711 driver](https://github.com/jancumps/pio_drv8711_stepper)  
[2 Stepper Motors on the same or different PIOs (example source)](https://gist.github.com/jancumps/c66e8af42dc30ee6dfbdfc06aea496e1)  
[Stepper Motor Control with Raspberry Pico PIO and A4988 driver](https://github.com/jancumps/pio_a4988_stepper)  

## toolchain requirements: 
- CMake 3.28 or higher
- GCC 14.2.1 or higher
- Pico C SDK 2.1.1
- tested with Pico 1 and Pico-W