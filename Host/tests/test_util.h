#ifndef TEST_UTIL_H
#define TEST_UTIL_H

/**
 * @file test_util.h
 * @brief 最朴素的断言驱动测试框架：计数 + 打印 + 非零退出码。
 *
 * 不引入第三方依赖（Unity 等），用 CTest 的返回码判定通过与否：
 * 任一 CHECK 失败 → 退出码非 0 → ctest 报 FAIL。
 * 每个测试 .c 只包含本头文件一次（计数器是 static）。
 */

#include <stdio.h>
#include <math.h>

static int g_checks_run = 0;
static int g_checks_failed = 0;

/* 相等断言（整型/布尔）。 */
#define CHECK(cond)                                                       \
    do                                                                    \
    {                                                                     \
        g_checks_run++;                                                   \
        if (!(cond))                                                      \
        {                                                                 \
            g_checks_failed++;                                            \
            printf("FAIL %s:%d  CHECK(%s)\n", __FILE__, __LINE__, #cond); \
        }                                                                 \
    } while (0)

/* 浮点近似断言：|a-b| <= tol。 */
#define CHECK_NEAR(a, b, tol)                                                     \
    do                                                                            \
    {                                                                             \
        double _va = (double)(a);                                                 \
        double _vb = (double)(b);                                                 \
        g_checks_run++;                                                           \
        if (!(fabs(_va - _vb) <= (double)(tol)))                                  \
        {                                                                         \
            g_checks_failed++;                                                    \
            printf("FAIL %s:%d  |%s - %s| = %.9g > %.9g\n", __FILE__, __LINE__,   \
                   #a, #b, fabs(_va - _vb), (double)(tol));                       \
        }                                                                         \
    } while (0)

/* 测试结束汇总：打印统计并返回退出码。 */
#define TEST_SUMMARY(name)                                        \
    do                                                            \
    {                                                             \
        printf("[%s] %d checks, %d failed -> %s\n", (name),       \
               g_checks_run, g_checks_failed,                    \
               g_checks_failed == 0 ? "PASS" : "FAIL");          \
        return g_checks_failed == 0 ? 0 : 1;                     \
    } while (0)

#endif /* TEST_UTIL_H */
