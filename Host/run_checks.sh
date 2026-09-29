#!/bin/sh
# ============================================================
# Host 一键本地检查（Phase 2 质量门禁）：静态分析 + 构建 + 单测 + 可选覆盖率
#
# 用法：
#   sh run_checks.sh                    # cppcheck + Ninja 构建 + ctest
#   CAR_COVERAGE=ON sh run_checks.sh    # 另加 gcov 覆盖率报告
#
# 路径说明（重要）：本仓库路径含中文，而 MinGW 工具链的两个已知坑都会被它触发：
#   1) libgcov 无法在非 ASCII 路径写 .gcda（覆盖率拿不到数据）；
#   2) 本机 ninja 的 FindFirstFileExA 以 ANSI(GBK) 枚举目录，对 UTF-8 与
#      混合分隔符路径偶发 ERROR_INVALID_NAME。
# 对策：脚本在 %TEMP% 建一个 NTFS junction（无需管理员），所有构建/测试/
# gcov 操作统一走 ASCII 路径 —— 已验证稳定。
#
# 退出码：0 = 全部通过；非 0 = 任一步骤失败（可直接接 CI）。
# ============================================================
set -e
cd "$(dirname "$0")"

echo "== [1/4] cppcheck 静态分析 =="
cppcheck --enable=warning,style,performance,portability --inline-suppr \
    --suppress=missingIncludeSystem --std=c99 \
    ../Service ../App ../Port sim -q
cppcheck --enable=warning --inline-suppr --suppress=missingIncludeSystem --std=c99 \
    ../BSP/bsp_adc.c ../BSP/bsp_encoder.c ../BSP/bsp_imu.c ../BSP/bsp_motor.c \
    ../BSP/bsp_uart.c -q
# 寄存器版互斥编译：单独展开 BSP_USE_REG=1 检查其内容
cppcheck --enable=warning --inline-suppr --suppress=missingIncludeSystem --std=c99 \
    -DBSP_USE_REG=1 ../BSP/bsp_imu_reg.c ../BSP/bsp_motor_reg.c -q
echo "   0 告警"

echo "== [2/4] 准备 ASCII junction =="
REAL="$(pwd -W 2>/dev/null || pwd)"
J="$(printf '%s' "$TEMP" | sed 's|\\|/|g')/car_hal_ascii"
if [ ! -e "$J/Host" ]; then
    powershell -NoProfile -Command \
        "New-Item -ItemType Junction -Path '$J' -Target '$REAL' -Force" >/dev/null
    echo "   已创建 $J -> $REAL"
else
    echo "   复用 $J"
fi
cd "$J/Host"

if [ "${CAR_COVERAGE:-OFF}" = "ON" ]; then
    BDIR=build-cov
else
    BDIR=build
fi

echo "== [3/4] 构建 + ctest + SIL 实验（$BDIR）=="
cmake -B "$BDIR" -G Ninja -DCAR_COVERAGE="${CAR_COVERAGE:-OFF}" >/dev/null
cmake --build "$BDIR" >/dev/null
ctest --test-dir "$BDIR" --output-on-failure
# 全量跑 E1~E5（含 --check 验收），数据落 $BDIR/sim_out 供 plot.py 出图
"./$BDIR/sil_sim" all "$BDIR/sim_out" --check

if [ "${CAR_COVERAGE:-OFF}" = "ON" ]; then
    echo "== [4/4] gcov 覆盖率（Service 层行覆盖）=="
    OBJDIR=$(dirname "$(find "$BDIR/CMakeFiles/car_service.dir" -name 'control_pid.c.gcda' | head -1)")
    (cd "$OBJDIR" && gcov -b ./*.gcda) | grep -E "^File|Lines executed" | paste - -
else
    echo "== [4/4] 覆盖率跳过（CAR_COVERAGE=ON 开启）=="
fi

echo "ALL CHECKS PASSED"
