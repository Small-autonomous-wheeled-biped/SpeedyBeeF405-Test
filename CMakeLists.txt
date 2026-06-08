cmake_minimum_required(VERSION 3.20)

project(speedybee_f405_imu C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(FIRMWARE_SOURCES
    src/startup_stm32f405xx.c
    src/board.c
    src/crc.c
    src/config_store.c
    src/icm42688p.c
    src/dps310.c
    src/imu_calibration.c
    src/imu_filters.c
    src/mahony_filter.c
    src/health_monitor.c
    src/fast_packet.c
    src/log_packet.c
    src/usb_cdc.c
    src/main.c
)

add_executable(${PROJECT_NAME} ${FIRMWARE_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    STM32F405xx
    HSE_VALUE=8000000UL
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Wshadow
    -Wdouble-promotion
    -Os
    -g3
)

target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -nostartfiles
    --specs=nosys.specs
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F405RGTx_FLASH.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)

# libm must come after objects so the linker resolves math references;
# nosys.specs provides __errno stub needed by asinf/powf/sqrtf.
target_link_libraries(${PROJECT_NAME} PRIVATE m)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    BYPRODUCTS
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
)
