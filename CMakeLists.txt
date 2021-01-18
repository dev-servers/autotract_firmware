cmake_minimum_required(VERSION 3.13)
set(CMAKE_TOOLCHAIN_FILE tools/stm32-cmake/cmake/stm32_gcc.cmake)
set(STM32_TOOLCHAIN_PATH ~/Work/Tools/gcc-arm-none-eabi-10)
project(firmware C CXX ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
set(STM32_CUBE_F4_PATH libs/stm32cube)
find_package(CMSIS COMPONENTS STM32F4 REQUIRED)
find_package(HAL COMPONENTS STM32F4 REQUIRED)
set(SOURCE_FILES src/main.cpp src/interrupts.cpp src/rosserial.cpp src/init.cpp src/errors.cpp src/steering.cpp src/stepper.cpp src/encoder.cpp libs/ros/time.cpp)

# Nucleo STM32F412ZG 
add_executable(nucleo_firmware.elf ${SOURCE_FILES})
target_compile_features(nucleo_firmware.elf  PUBLIC cxx_std_17)
target_include_directories(
    nucleo_firmware.elf
    PUBLIC
    inc/
    libs/ros/
)
target_link_libraries(nucleo_firmware.elf 
    HAL::STM32::F4::RCC
    HAL::STM32::F4::GPIO
    HAL::STM32::F4::CORTEX
    HAL::STM32::F4::UART
    HAL::STM32::F4::DMA
    CMSIS::STM32::F412ZG
    STM32::NoSys
)
target_compile_definitions(nucleo_firmware.elf PUBLIC NUCLEO)

# BlackPill STM32F411CE
add_executable(blackpill_firmware.elf ${SOURCE_FILES})

target_compile_features(blackpill_firmware.elf  PUBLIC cxx_std_17)
target_include_directories(
    blackpill_firmware.elf
    PUBLIC
    inc/
    libs/ros/
)
target_link_libraries(blackpill_firmware.elf 
    HAL::STM32::F4::RCC
    HAL::STM32::F4::GPIO
    HAL::STM32::F4::CORTEX
    HAL::STM32::F4::UART
    HAL::STM32::F4::DMA
    CMSIS::STM32::F411CE
    STM32::NoSys
)
target_compile_definitions(blackpill_firmware.elf PUBLIC BLACKPILL)