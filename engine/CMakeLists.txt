cmake_minimum_required(VERSION 3.18)
project(orbitforge_engine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(ENABLE_TSAN "Enable ThreadSanitizer (disables ASan)" OFF)

# Compiler flags
add_compile_options(-O2 -Wall -Wextra -Wno-unused-parameter)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(ENABLE_TSAN)
        add_compile_options(-g -fsanitize=thread)
        add_link_options(-fsanitize=thread)
    else()
        add_compile_options(-g -fsanitize=address,undefined)
        add_link_options(-fsanitize=address,undefined)
    endif()
endif()

# Eigen3
find_package(Eigen3 REQUIRED)

# Threads (ring buffer multi-thread test, Phase 2)
find_package(Threads REQUIRED)

# GTest via FetchContent
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include_directories(include)

# Engine source files
set(ENGINE_SOURCES
    src/dynamics/eom.cpp
    src/dynamics/perturbations.cpp
    src/dynamics/rigid_body.cpp
    src/math/quaternion.cpp
    src/filters/kf.cpp
    src/filters/ekf.cpp
    src/filters/ukf.cpp
    src/sensors/magnetometer.cpp
    src/wasm_api.cpp
    src/monte_carlo/mc_runner.cpp
)

# Test target
add_executable(orbitforge_tests
    ${ENGINE_SOURCES}
    tests/test_eom.cpp
    tests/test_rk4.cpp
    tests/test_rk45.cpp
    tests/test_quaternion.cpp
    tests/test_rigid_body.cpp
    tests/test_kf.cpp
    tests/test_ekf.cpp
    tests/test_ukf.cpp
    tests/test_sensors.cpp
    tests/test_filter_consistency.cpp
    tests/test_ring_buffer.cpp
    tests/test_pool_alloc.cpp
    tests/test_scenario.cpp
    tests/test_fault_injector.cpp
    tests/test_wasm_api.cpp
    tests/test_ensemble.cpp
    tests/test_mc_runner.cpp
)
target_include_directories(orbitforge_tests PRIVATE include)
target_link_libraries(orbitforge_tests PRIVATE GTest::gtest_main Eigen3::Eigen Threads::Threads)
enable_testing()
include(GoogleTest)
gtest_discover_tests(orbitforge_tests)

# Benchmark target
add_executable(orbitforge_benchmarks
    ${ENGINE_SOURCES}
    benchmarks/bench_integrator.cpp
    benchmarks/bench_filters.cpp
    benchmarks/bench_monte_carlo.cpp
)
target_include_directories(orbitforge_benchmarks PRIVATE include)
target_link_libraries(orbitforge_benchmarks PRIVATE Eigen3::Eigen Threads::Threads)
