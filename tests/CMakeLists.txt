add_executable(rcj_control_tests
    control_tests.c
    ../firmware/stm32_chassis/App/Src/app_lqr.c
    ../firmware/stm32_chassis/App/Src/app_velocity_command.c
    ../apps/soccer_sim/soccer_predict.c
)
target_include_directories(rcj_control_tests PRIVATE
    ../firmware/stm32_chassis/App/Inc
    ../firmware/stm32_chassis/Bsp/Inc
    ../apps/soccer_sim
)
target_link_libraries(rcj_control_tests PRIVATE m)
target_compile_options(rcj_control_tests PRIVATE -Wall -Wextra -Werror)
add_test(NAME rcj_control_tests COMMAND rcj_control_tests)
