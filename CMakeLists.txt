cmake_minimum_required(VERSION 3.28)

include(pico_sdk_import.cmake)

project(stepper C CXX ASM)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmodules-ts -fcommon")

pico_sdk_init()

include(FetchContent)

FetchContent_Declare(pio_irq
  GIT_REPOSITORY "https://github.com/jancumps/pio_irq.git"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(pio_irq)

add_library(stepper)
target_sources(stepper
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/stepper_impl.cpp
)
target_sources(stepper
        PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
#        ${CMAKE_CURRENT_SOURCE_DIR}/source/pio_irq_util.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/stepper.cpp
)
pico_generate_pio_header(stepper
        ${CMAKE_CURRENT_SOURCE_DIR}/source/stepper.pio
)

target_link_libraries( stepper
        pico_stdlib
        pio_irq
        $<COMPILE_ONLY:hardware_pio>
        $<COMPILE_ONLY:hardware_gpio>
)

