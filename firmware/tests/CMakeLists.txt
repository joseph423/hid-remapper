cmake_minimum_required(VERSION 3.13)

project(tps43_host_tests LANGUAGES CXX)

enable_testing()

add_executable(dual_tps43_architecture_test
    dual_tps43_architecture_test.cc
    ../src/dual_tps43_coordinator.cc
    ../src/dual_tps43_fsm.cc
    ../src/pad_state.cc
)
target_compile_features(dual_tps43_architecture_test PRIVATE cxx_std_17)
target_compile_options(dual_tps43_architecture_test PRIVATE -Wall -Wextra -Werror)
target_include_directories(dual_tps43_architecture_test PRIVATE ../src)

add_test(NAME dual_tps43_architecture_test COMMAND dual_tps43_architecture_test)

add_executable(dual_tps43_fsm_test
    dual_tps43_fsm_test.cc
    ../src/dual_tps43_fsm.cc
    ../src/pad_state.cc
)
target_compile_features(dual_tps43_fsm_test PRIVATE cxx_std_17)
target_compile_options(dual_tps43_fsm_test PRIVATE -Wall -Wextra -Werror)
target_include_directories(dual_tps43_fsm_test PRIVATE ../src)

add_test(NAME dual_tps43_fsm_test COMMAND dual_tps43_fsm_test)

add_executable(dual_tps43_motion_test
    dual_tps43_motion_test.cc
    ../src/dual_tps43_fsm.cc
    ../src/pad_state.cc
)
target_compile_features(dual_tps43_motion_test PRIVATE cxx_std_17)
target_compile_options(dual_tps43_motion_test PRIVATE -Wall -Wextra -Werror)
target_include_directories(dual_tps43_motion_test PRIVATE ../src)

add_test(NAME dual_tps43_motion_test COMMAND dual_tps43_motion_test)

add_executable(tps43_timing_processor_test
    tps43_timing_processor_test.cc
    ../src/tps43_timing_processor.cc
)
target_compile_features(tps43_timing_processor_test PRIVATE cxx_std_17)
target_compile_options(tps43_timing_processor_test PRIVATE -Wall -Wextra -Werror)
target_include_directories(tps43_timing_processor_test PRIVATE ../src)

add_test(NAME tps43_timing_processor_test COMMAND tps43_timing_processor_test)

add_executable(tps43_async_driver_test
    tps43_async_driver_test.cc
    ../src/tps43_iqs5xx_driver.cc
)
target_compile_features(tps43_async_driver_test PRIVATE cxx_std_17)
target_compile_options(tps43_async_driver_test PRIVATE -Wall -Wextra -Werror)
target_include_directories(tps43_async_driver_test PRIVATE fake_pico ../src)
add_test(NAME tps43_async_driver_test COMMAND tps43_async_driver_test)
set_tests_properties(tps43_async_driver_test PROPERTIES TIMEOUT 5)

add_executable(tps43_normal_capture_test
    tps43_normal_capture_test.cc
    ../src/tps43_timing_metrics.cc
)
target_compile_features(tps43_normal_capture_test PRIVATE cxx_std_17)
target_compile_options(tps43_normal_capture_test PRIVATE -Wall -Wextra -Werror)
target_include_directories(tps43_normal_capture_test PRIVATE ../src)
add_test(NAME tps43_normal_capture_test COMMAND tps43_normal_capture_test)

add_executable(tps43_capture_commands_test
    tps43_capture_commands_test.cc
    ../src/tps43_timing_capture.cc
    ../src/tps43_timing_metrics.cc
)
target_compile_features(tps43_capture_commands_test PRIVATE cxx_std_17)
target_compile_options(tps43_capture_commands_test PRIVATE -Wall -Wextra -Werror)
target_include_directories(tps43_capture_commands_test PRIVATE fake_pico ../src)
add_test(NAME tps43_capture_commands_test COMMAND tps43_capture_commands_test)
