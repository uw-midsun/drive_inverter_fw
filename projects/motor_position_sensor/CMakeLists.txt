cmake_minimum_required(VERSION 3.22)

project(motor_position_sensor LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # clangd / editor tooling

include(stm32_app)

stm32_firmware(
    NAME           motor_position_sensor
    HAL_FAMILY     STM32F1xx
    CMSIS_DEVICE   STM32F1xx
    STARTUP        Core/Startup/startup_stm32f103cbtx.s
    LINKER_SCRIPT  STM32F103CBTX_FLASH.ld
    OPENOCD_TARGET stm32f1x.cfg
    DEFINES        USE_HAL_DRIVER STM32F103xB
)
