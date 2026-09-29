# ============================================================
# Host 侧单元测试工程（Phase 2）
#
# 目标：把 Service/ 纯 C 算法编译成 PC 原生库并跑单元测试——
# 与固件共享同一份源文件，这就是"可测试性设计（Design for Test）"：
#   固件: Service/*.c ── armcc ──→ STM32
#   主机: Service/*.c ── gcc   ──→ 本目录测试/仿真（Phase 3）
#
# 用法：
#   cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure
# 覆盖率：
#   cmake -B build-cov -DCAR_COVERAGE=ON && cmake --build build-cov
#   ctest --test-dir build-cov && gcov ...（见 run_checks.sh）
# ============================================================
cmake_minimum_required(VERSION 3.16)
project(car_host LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(CAR_COVERAGE "Enable gcov coverage instrumentation" OFF)

# 全局告警即错误：主机侧代码质量门禁与固件同级
add_compile_options(-Wall -Wextra -Werror)

# ---- Service 静态库（与固件同源，零硬件依赖） ----
add_library(car_service STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../Service/control_pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Service/attitude_kalman.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Service/protocol.c
)
target_include_directories(car_service PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../Service
    ${CMAKE_CURRENT_SOURCE_DIR}/../App   # Car_Config.h（纯宏配置头）
)

if(CAR_COVERAGE)
    target_compile_options(car_service PRIVATE --coverage)
    target_link_options(car_service PUBLIC --coverage)
endif()

enable_testing()

# ---- 单元测试：每个测试一个可执行文件，返回码即通过与否 ----
foreach(t control attitude protocol)
    add_executable(test_${t} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_${t}.c)
    target_link_libraries(test_${t} PRIVATE car_service)
    if(NOT WIN32)
        target_link_libraries(test_${t} PRIVATE m)
    endif()
    add_test(NAME ${t} COMMAND test_${t})
endforeach()

# ============================================================
# SIL 固件在环仿真（Phase 3）：真实 App+Service 代码 × port_sim 倒立摆
# 与单片机共享同一份 App_Car.c / Service/*.c，只换 Port 实现。
# ============================================================
add_executable(sil_sim
    ${CMAKE_CURRENT_SOURCE_DIR}/sim/main_sil.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sim/pendulum_model.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/App_Car.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Port/port_sim.c
)
target_include_directories(sil_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}          # sim/pendulum_model.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../Port  # port_sim.h
)
# 主机侧：App 层的临界区宏走空实现（见 App/car_critical.h）
target_compile_definitions(sil_sim PRIVATE CAR_HOST_BUILD)
target_link_libraries(sil_sim PRIVATE car_service)
if(NOT WIN32)
    target_link_libraries(sil_sim PRIVATE m)
endif()

# 安全与闭环验收进 CTest 门禁（--check 按验收标准决定退出码）；
# E3~E5 属实验性产出（CSV/图表），由 tools/plot.py 消费，不设硬门禁。
add_test(NAME sil_e1 COMMAND sil_sim e1 sim_out --check)
add_test(NAME sil_e2 COMMAND sil_sim e2 sim_out --check)
add_test(NAME sil_e6 COMMAND sil_sim e6 sim_out --check)
add_test(NAME sil_e7 COMMAND sil_sim e7 sim_out --check)
