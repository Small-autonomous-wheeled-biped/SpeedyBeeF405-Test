cmake_minimum_required(VERSION 3.20)

project(speedybee_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

# Shared compile options.
set(HOST_COMPILE_OPTIONS
    -Wall
    -Wextra
    -Wshadow
)

# Common source files that are pure-C (no hardware dependencies).
set(PURE_C_SOURCES
    ../../src/crc.c
    ../../src/config_store.c
    ../../src/mahony_filter.c
    ../../src/imu_calibration.c
    ../../src/imu_filters.c
    ../../src/fast_packet.c
    ../../src/log_packet.c
    ../../src/health_monitor.c
)

# Driver sources that depend on board (injected via fake_board).
set(DRIVER_SOURCES
    ../../src/icm42688p.c
    ../../src/dps310.c
)

# ---- test_drivers (driver behavior via fake_board) ----------------------
add_executable(test_drivers
    fake_board.c
    test_drivers.c
    ${DRIVER_SOURCES}
)
target_include_directories(test_drivers PRIVATE . ../../include)
target_compile_options(test_drivers PRIVATE ${HOST_COMPILE_OPTIONS})
target_compile_definitions(test_drivers PRIVATE HOST_TEST)
target_link_libraries(test_drivers m)
add_test(NAME test_drivers COMMAND test_drivers)

# ---- test_crc -----------------------------------------------------------
add_executable(test_crc
    test_crc.c
    ../../src/crc.c
)
target_include_directories(test_crc PRIVATE ../../include)
target_compile_options(test_crc PRIVATE ${HOST_COMPILE_OPTIONS})
target_compile_definitions(test_crc PRIVATE HOST_TEST)
add_test(NAME test_crc COMMAND test_crc)

# ---- test_mahony --------------------------------------------------------
add_executable(test_mahony
    test_mahony.c
    ../../src/mahony_filter.c
)
target_include_directories(test_mahony PRIVATE ../../include)
target_compile_options(test_mahony PRIVATE ${HOST_COMPILE_OPTIONS})
target_compile_definitions(test_mahony PRIVATE HOST_TEST)
target_link_libraries(test_mahony m)
add_test(NAME test_mahony COMMAND test_mahony)

# ---- test_packets -------------------------------------------------------
add_executable(test_packets
    test_packets.c
    ../../src/fast_packet.c
    ../../src/log_packet.c
    ../../src/crc.c
)
target_include_directories(test_packets PRIVATE ../../include)
target_compile_options(test_packets PRIVATE ${HOST_COMPILE_OPTIONS})
target_compile_definitions(test_packets PRIVATE HOST_TEST)
target_link_libraries(test_packets m)
add_test(NAME test_packets COMMAND test_packets)
