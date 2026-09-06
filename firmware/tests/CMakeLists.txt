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
