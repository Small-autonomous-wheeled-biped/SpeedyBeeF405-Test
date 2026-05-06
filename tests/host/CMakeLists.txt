cmake_minimum_required(VERSION 3.20)

project(speedybee_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(driver_tests
    fake_board.c
    test_drivers.c
    ../../src/icm42688p.c
    ../../src/dps310.c
)

target_include_directories(driver_tests PRIVATE
    .
    ../../include
)

target_compile_options(driver_tests PRIVATE
    -Wall
    -Wextra
    -Wshadow
)

add_test(NAME driver_tests COMMAND driver_tests)
