# Shared firmware target builder for drive inverter STM32 projects
#
#   stm32_firmware(
#     NAME           motor_controller
#     HAL_FAMILY     STM32G4xx                  # Drivers/<HAL_FAMILY>_HAL_Driver
#     CMSIS_DEVICE   STM32G4xx                  # Drivers/CMSIS/Device/ST/<CMSIS_DEVICE>/Include
#     STARTUP        startup_stm32g474xx.s
#     LINKER_SCRIPT  STM32G474XX_FLASH.ld
#     OPENOCD_TARGET stm32g4x.cfg
#     DEFINES        USE_HAL_DRIVER USE_FULL_LL_DRIVER STM32G474xx
#   )

function(stm32_firmware)
    cmake_parse_arguments(FW
        ""
        "NAME;HAL_FAMILY;CMSIS_DEVICE;STARTUP;LINKER_SCRIPT;OPENOCD_TARGET"
        "DEFINES"
        ${ARGN})

    set(ELF ${FW_NAME}.elf)
    add_executable(${ELF})

    set(HAL_DIR ${CMAKE_SOURCE_DIR}/Drivers/${FW_HAL_FAMILY}_HAL_Driver)

    file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_SOURCE_DIR}/Core/Src/*.c
        ${HAL_DIR}/Src/*.c
    )
    list(FILTER APP_SOURCES EXCLUDE REGEX ".*_template\\.c$")

    target_sources(${ELF} PRIVATE
        ${APP_SOURCES}
        ${CMAKE_SOURCE_DIR}/${FW_STARTUP}
    )

    # Every directory under Core/Src/app that holds a header becomes an include
    # dir so app modules can include siblings by bare name
    file(GLOB_RECURSE APP_HEADERS CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/Core/Src/app/*.h)
    set(APP_INCLUDE_DIRS "")
    foreach(_h ${APP_HEADERS})
        get_filename_component(_d ${_h} DIRECTORY)
        list(APPEND APP_INCLUDE_DIRS ${_d})
    endforeach()
    if(APP_INCLUDE_DIRS)
        list(REMOVE_DUPLICATES APP_INCLUDE_DIRS)
    endif()

    target_include_directories(${ELF} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${APP_INCLUDE_DIRS}
        ${HAL_DIR}/Inc
        ${HAL_DIR}/Inc/Legacy
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/${FW_CMSIS_DEVICE}/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    )

    target_compile_definitions(${ELF} PRIVATE
        ${FW_DEFINES}
        $<$<CONFIG:Debug>:DEBUG>
    )

    target_compile_options(${ELF} PRIVATE
        -Wall -Wextra
        -ffunction-sections -fdata-sections
        $<$<CONFIG:Debug>:-Og -g3>
        $<$<CONFIG:Release>:-O2 -g0>
    )

    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/${FW_LINKER_SCRIPT})
    target_link_options(${ELF} PRIVATE
        -T${LINKER_SCRIPT}
        --specs=nano.specs --specs=nosys.specs
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map=${FW_NAME}.map,--cref
    )

    target_link_libraries(${ELF} PRIVATE m)
    set_target_properties(${ELF} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

    add_custom_command(TARGET ${ELF} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${ELF}> ${FW_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${ELF}> ${FW_NAME}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${ELF}>
        VERBATIM
    )

    # Probe must be bridged into WSL first (Windows admin PowerShell, per session):
    #   usbipd attach --wsl --busid <id>
    set(OPENOCD_CFG -f interface/cmsis-dap.cfg -f target/${FW_OPENOCD_TARGET})

    add_custom_target(flash
        COMMAND openocd ${OPENOCD_CFG}
            -c "program $<TARGET_FILE:${ELF}> verify reset exit"
        DEPENDS ${ELF}
        USES_TERMINAL
        COMMENT "Flashing $<TARGET_FILE:${ELF}> via CMSIS-DAP (OpenOCD)"
    )

    add_custom_target(openocd
        COMMAND openocd ${OPENOCD_CFG}
        USES_TERMINAL
        COMMENT "OpenOCD GDB server on :3333, attach: target extended-remote :3333"
    )

    add_custom_target(debug
        COMMAND arm-none-eabi-gdb $<TARGET_FILE:${ELF}>
            -ex "target extended-remote | openocd ${OPENOCD_CFG} -c \"gdb_port pipe\""
            -ex "monitor reset halt" -ex "load" -ex "break main" -ex "continue"
        DEPENDS ${ELF}
        USES_TERMINAL
        COMMENT "Flash + gdb debug session over CMSIS-DAP"
    )
endfunction()
