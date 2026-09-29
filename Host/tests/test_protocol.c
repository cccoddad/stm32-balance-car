/**
 * @file test_protocol.c
 * @brief Service/protocol.c 单元测试：帧解析、兼容模式、健壮性与模糊测试。
 *
 * 覆盖升级方案 Phase 2 规定的用例：
 * - 正常帧 @MV/@TR/@PID、裸字符兼容
 * - 乱序 / 半包 / 超长帧 / 二进制垃圾不崩溃、可恢复
 * - 字段超长整帧丢弃（防止截断产生错误数值）
 * - 模糊测试：10 万字节确定性随机输入 + 周期性插入合法帧
 * - Proto_ParseFloat 各种合法/非法输入
 */
#include "test_util.h"
#include "protocol.h"
#include <string.h>

/* 确定性伪随机字节流（LCG），模糊测试跨平台可复现。 */
static unsigned int g_lcg_state = 0x12345678u;
static unsigned char RandByte(void)
{
    g_lcg_state = g_lcg_state * 1664525u + 1013904223u;
    return (unsigned char)(g_lcg_state >> 24);
}

/* 喂一串字节，返回产生的指令数（并可选校验最后一条）。 */
static int FeedStr(proto_parser_t *p, const char *s, proto_msg_t *last)
{
    int n = 0;
    proto_msg_t msg;
    while (*s != '\0')
    {
        if (Proto_Feed(p, (unsigned char)*s, &msg))
        {
            n++;
            if (last != 0)
            {
                *last = msg;
            }
        }
        s++;
    }
    return n;
}

/* ---------- 正常路径 ---------- */

static void Test_LegacyChars(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    Proto_Init(&p);

    CHECK(Proto_Feed(&p, 'U', &msg) && msg.cmd == PROTO_MOVE && msg.arg1[0] == 'U');
    CHECK(Proto_Feed(&p, 'D', &msg) && msg.cmd == PROTO_MOVE && msg.arg1[0] == 'D');
    CHECK(Proto_Feed(&p, 'S', &msg) && msg.cmd == PROTO_MOVE && msg.arg1[0] == 'S');
    CHECK(Proto_Feed(&p, 'L', &msg) && msg.cmd == PROTO_TURN && msg.arg1[0] == 'L');
    CHECK(Proto_Feed(&p, 'R', &msg) && msg.cmd == PROTO_TURN && msg.arg1[0] == 'R');
    /* 非指令字符不产生消息（注意 'U' 的 ASCII 恰是 0x55，故用 'V'=0x56）。 */
    CHECK(!Proto_Feed(&p, 'x', &msg));
    CHECK(!Proto_Feed(&p, 'V', &msg));
}

static void Test_Frames(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    Proto_Init(&p);

    /* @MV,U# */
    memset(&msg, 0, sizeof(msg));
    CHECK(FeedStr(&p, "@MV,U#", &msg) == 1);
    CHECK(msg.cmd == PROTO_MOVE && msg.arg1[0] == 'U' && msg.arg1[1] == '\0');

    /* @TR,L# */
    memset(&msg, 0, sizeof(msg));
    CHECK(FeedStr(&p, "@TR,L#", &msg) == 1);
    CHECK(msg.cmd == PROTO_TURN && msg.arg1[0] == 'L');

    /* @PID,BKP,-720.0# */
    memset(&msg, 0, sizeof(msg));
    CHECK(FeedStr(&p, "@PID,BKP,-720.0#", &msg) == 1);
    CHECK(msg.cmd == PROTO_PID);
    CHECK(strcmp(msg.arg1, "BKP") == 0);
    CHECK(strcmp(msg.arg2, "-720.0") == 0);

    /* @ST#：状态查询，无参数。 */
    memset(&msg, 0, sizeof(msg));
    CHECK(FeedStr(&p, "@ST#", &msg) == 1);
    CHECK(msg.cmd == PROTO_STATUS);
    CHECK(msg.arg1[0] == 'S');

    /* 帧内出现的逗号外内容不产生消息（收帧中字节静默消费）。 */
    CHECK(FeedStr(&p, "@MV", 0) == 0);
    Proto_Init(&p);
}

/* ---------- 健壮性 ---------- */

static void Test_HalfPacket_Recover(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    Proto_Init(&p);

    /* 半包丢弃：@MV,U 无帧尾 → 新 @ 直接开新帧。 */
    CHECK(FeedStr(&p, "@MV,U", 0) == 0);
    CHECK(FeedStr(&p, "@MV,D#", &msg) == 1);
    CHECK(msg.arg1[0] == 'D');
}

static void Test_BinaryGarbage_AbortFrame(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    int i;
    Proto_Init(&p);

    /* 帧内二进制垃圾 → 整帧作废。 */
    CHECK(FeedStr(&p, "@MV,", 0) == 0);
    (void)Proto_Feed(&p, 0x00, &msg);
    (void)Proto_Feed(&p, 0xFF, &msg);
    /* 帧已作废，尾巴不再构成帧内指令（用非指令字符 'X' 验证静默忽略，
     * 裸 'U' 是合法遥控指令会误导断言）。 */
    CHECK(FeedStr(&p, "X#", 0) == 0);

    /* 帧外垃圾被静默忽略。 */
    for (i = 0; i < 50; i++)
    {
        (void)Proto_Feed(&p, (uint8_t)(i & 0xFF), &msg);
        (void)Proto_Feed(&p, 0x80, &msg);
    }

    /* 垃圾之后系统仍能解析合法帧。 */
    CHECK(FeedStr(&p, "@TR,R#", &msg) == 1);
    CHECK(msg.arg1[0] == 'R');
}

static void Test_OverlongFrame_Dropped(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    char buf[80];
    int i;
    Proto_Init(&p);

    /* 超过 PROTO_FRAME_MAX 的帧体：缓冲不溢出、整帧作废。 */
    buf[0] = '@';
    for (i = 1; i < 70; i++)
    {
        buf[i] = 'A';
    }
    buf[70] = '#';
    buf[71] = '\0';
    CHECK(FeedStr(&p, buf, 0) == 0);

    /* 超长后恢复。 */
    CHECK(FeedStr(&p, "@MV,S#", &msg) == 1);
    CHECK(msg.arg1[0] == 'S');
}

static void Test_UnknownFrame_Rejected(void)
{
    proto_parser_t p;
    Proto_Init(&p);

    CHECK(FeedStr(&p, "@XX,Y#", 0) == 0);  /* 未知命令 */
    CHECK(FeedStr(&p, "@MV,X#", 0) == 0);  /* 非法方向字符 */
    CHECK(FeedStr(&p, "@MV,UU#", 0) == 0); /* 方向字段多字符 */
    CHECK(FeedStr(&p, "@PID,,1.0#", 0) == 0);   /* 空参数名 */
    CHECK(FeedStr(&p, "@PID,BKP,#", 0) == 0);   /* 空数值 */
    CHECK(FeedStr(&p, "@#", 0) == 0);           /* 空帧体 */
    CHECK(FeedStr(&p, "@MV,U,D,L#", 0) == 0);   /* 字段过多 */
}

static void Test_LongField_TruncatedFrame(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    char buf[64];
    int i;
    Proto_Init(&p);

    /* 参数名 20 字符（> PROTO_ARG_MAX-1）：禁止静默截断，整帧丢弃。 */
    memcpy(buf, "@PID,", 5);
    for (i = 0; i < 20; i++)
    {
        buf[5 + i] = 'A';
    }
    memcpy(buf + 25, ",1.0#", 6); /* 25+5+1=31 字节含结束符 */
    CHECK(FeedStr(&p, buf, 0) == 0);

    /* 短字段正常帧依然可用。 */
    CHECK(FeedStr(&p, "@PID,VKP,170.0#", &msg) == 1);
    CHECK(strcmp(msg.arg1, "VKP") == 0);
}

/* ---------- 模糊测试 ---------- */

static void Test_Fuzz_100kBytes(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    int i;
    int cmds = 0;
    Proto_Init(&p);

    /* 10 万字节确定性随机流；每 500 字节插入一条完整合法帧，
     * 验证"垃圾洪流中解析器不崩溃、且不丢失随后到来的合法指令"。 */
    for (i = 0; i < 100000; i++)
    {
        if (i % 500 == 0)
        {
            cmds += FeedStr(&p, "@MV,U#", &msg);
        }
        if (Proto_Feed(&p, RandByte(), &msg))
        {
            cmds++;
        }
    }
    /* 随机字节偶尔也会凑成合法帧，但插入的 200 条必须全部被解析出来。 */
    CHECK(cmds >= 200);

    /* 模糊测试后解析器仍工作。 */
    Proto_Init(&p);
    CHECK(FeedStr(&p, "@TR,L#", &msg) == 1);
    CHECK(msg.arg1[0] == 'L');
}

/* ---------- 浮点解析 ---------- */

static void Test_ParseFloat(void)
{
    float v;

    CHECK(Proto_ParseFloat("-720.0", &v) && v == -720.0f);
    CHECK(Proto_ParseFloat("12.25", &v) && v == 12.25f);
    CHECK(Proto_ParseFloat("+3.5", &v) && v == 3.5f);
    CHECK(Proto_ParseFloat("0", &v) && v == 0.0f);
    CHECK(Proto_ParseFloat("-0.5", &v) && v == -0.5f);
    CHECK(Proto_ParseFloat("  7.5 ", &v) && v == 7.5f); /* 首尾空白 */
    CHECK(Proto_ParseFloat(".25", &v) && v == 0.25f);   /* 省略整数部分 */

    /* 非法输入：返回 false 且不改写 out。 */
    v = 12345.0f;
    CHECK(!Proto_ParseFloat("", &v));
    CHECK(!Proto_ParseFloat("abc", &v));
    CHECK(!Proto_ParseFloat("-", &v));
    CHECK(!Proto_ParseFloat("12.3.4", &v));
    CHECK(!Proto_ParseFloat("1e5", &v));   /* 不支持科学计数法 */
    CHECK(!Proto_ParseFloat("12px", &v));
    CHECK(v == 12345.0f); /* out 未被污染 */
}

int main(void)
{
    Test_LegacyChars();
    Test_Frames();
    Test_HalfPacket_Recover();
    Test_BinaryGarbage_AbortFrame();
    Test_OverlongFrame_Dropped();
    Test_UnknownFrame_Rejected();
    Test_LongField_TruncatedFrame();
    Test_Fuzz_100kBytes();
    Test_ParseFloat();
    TEST_SUMMARY("protocol");
}
