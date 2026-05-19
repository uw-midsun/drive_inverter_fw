cmake_minimum_required(VERSION 3.22)

project(motor_controller LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # clangd / editor tooling

include(stm32_app)

stm32_firmware(
    NAME           motor_controller
    HAL_FAMILY     STM32G4xx
    CMSIS_DEVICE   STM32G4xx
    STARTUP        startup_stm32g474xx.s
    LINKER_SCRIPT  STM32G474XX_FLASH.ld
    OPENOCD_TARGET stm32g4x.cfg
    DEFINES        USE_HAL_DRIVER USE_FULL_LL_DRIVER STM32G474xx
)
