#include "protocol.h"

/**
 * @brief 安全拷贝以 '\0' 结尾的字符串到定长缓冲区（自动截断并保证结束符）。
 */
static void Proto_CopyArg(char *dst, unsigned dst_size, const char *src)
{
    unsigned i = 0;
    if (dst_size == 0)
    {
        return;
    }
    while (i < dst_size - 1u && src[i] != '\0')
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

/**
 * @brief 填充一条指令消息。
 */
static void Proto_SetMsg(proto_msg_t *msg, proto_cmd_t cmd, const char *arg1, const char *arg2)
{
    msg->cmd = cmd;
    Proto_CopyArg(msg->arg1, sizeof(msg->arg1), arg1);
    msg->arg2[0] = '\0';
    if (arg2 != 0)
    {
        Proto_CopyArg(msg->arg2, sizeof(msg->arg2), arg2);
    }
}

void Proto_Init(proto_parser_t *p)
{
    p->len = 0;
    p->in_frame = 0;
    p->buf[0] = '\0';
}

/**
 * @brief 按逗号切分帧体并生成指令。
 *
 * 帧体格式：CMD[,arg1[,arg2]]。字段数与命令类型不匹配时整帧丢弃。
 */
static bool Proto_ParseFrame(const char *frame, proto_msg_t *msg)
{
    /* 手工切分：最多 3 个字段，全部落在栈上定长数组，不调用 strtok（非重入）。 */
    char tok[3][PROTO_ARG_MAX];
    unsigned ntok = 0;
    unsigned i = 0;

    while (ntok < 3u)
    {
        unsigned j = 0;
        int truncated = 0;
        while (frame[i] != '\0' && frame[i] != ',')
        {
            if (j < PROTO_ARG_MAX - 1u)
            {
                tok[ntok][j++] = frame[i];
            }
            else
            {
                truncated = 1; /* 字段超长：静默截断会得到错误数值，整帧丢弃 */
            }
            i++;
        }
        tok[ntok][j] = '\0';
        if (truncated)
        {
            return false;
        }
        ntok++;
        if (frame[i] == ',')
        {
            i++;
        }
        else
        {
            break; /* 帧体结束 */
        }
    }
    /* 剩余还有内容说明字段过多，丢帧。 */
    if (frame[i] != '\0')
    {
        return false;
    }

    if (tok[0][0] == 'M' && tok[0][1] == 'V' && tok[0][2] == '\0' && ntok >= 2u)
    {
        /* @MV,U/D/S# */
        if (tok[1][1] != '\0')
        {
            return false;
        }
        if (tok[1][0] != 'U' && tok[1][0] != 'D' && tok[1][0] != 'S')
        {
            return false;
        }
        Proto_SetMsg(msg, PROTO_MOVE, tok[1], 0);
        return true;
    }
    if (tok[0][0] == 'T' && tok[0][1] == 'R' && tok[0][2] == '\0' && ntok >= 2u)
    {
        /* @TR,L/R/S# */
        if (tok[1][1] != '\0')
        {
            return false;
        }
        if (tok[1][0] != 'L' && tok[1][0] != 'R' && tok[1][0] != 'S')
        {
            return false;
        }
        Proto_SetMsg(msg, PROTO_TURN, tok[1], 0);
        return true;
    }
    if (tok[0][0] == 'P' && tok[0][1] == 'I' && tok[0][2] == 'D' && tok[0][3] == '\0' && ntok >= 3u)
    {
        /* @PID,<参数名>,<数值># */
        if (tok[1][0] == '\0' || tok[2][0] == '\0')
        {
            return false;
        }
        Proto_SetMsg(msg, PROTO_PID, tok[1], tok[2]);
        return true;
    }
    return false;
}

bool Proto_Feed(proto_parser_t *p, uint8_t byte, proto_msg_t *msg)
{
    if (byte == '@')
    {
        /* 帧头：无论是否处于收帧状态都重新开始（半包自愈）。 */
        p->in_frame = 1;
        p->len = 0;
        p->buf[0] = '\0';
        return false;
    }

    if (!p->in_frame)
    {
        /* 裸字符遥控模式：兼容课程配套蓝牙助手的单字符指令。 */
        switch (byte)
        {
        case 'U':
        case 'D':
        case 'S':
        {
            char arg[2];
            arg[0] = (char)byte;
            arg[1] = '\0';
            Proto_SetMsg(msg, PROTO_MOVE, arg, 0);
            return true;
        }
        case 'L':
        case 'R':
        {
            char arg[2];
            arg[0] = (char)byte;
            arg[1] = '\0';
            Proto_SetMsg(msg, PROTO_TURN, arg, 0);
            return true;
        }
        default:
            return false; /* 其余字节（含二进制垃圾）一律忽略 */
        }
    }

    /* ---- 收帧状态 ---- */
    if (byte == '#')
    {
        /* 帧尾：组装解析。无论成功与否都结束本帧。 */
        p->in_frame = 0;
        p->buf[p->len] = '\0';
        return Proto_ParseFrame(p->buf, msg);
    }
    if (byte < 0x20u || byte > 0x7Eu)
    {
        /* 帧内出现控制字符/二进制垃圾：整帧作废，防止解析出诡异指令。 */
        p->in_frame = 0;
        p->len = 0;
        p->buf[0] = '\0';
        return false;
    }
    if (p->len >= PROTO_FRAME_MAX - 1u)
    {
        /* 超长帧：作废，防止缓冲区溢出。 */
        p->in_frame = 0;
        p->len = 0;
        p->buf[0] = '\0';
        return false;
    }
    p->buf[p->len++] = (char)byte;
    return false;
}

bool Proto_ParseFloat(const char *s, float *out)
{
    unsigned i = 0;
    int sign = 1;
    double value = 0.0;
    double frac = 0.0;
    double div = 1.0;
    int any_digit = 0;

    while (s[i] == ' ')
    {
        i++;
    }
    if (s[i] == '+')
    {
        i++;
    }
    else if (s[i] == '-')
    {
        sign = -1;
        i++;
    }
    while (s[i] >= '0' && s[i] <= '9')
    {
        value = value * 10.0 + (double)(s[i] - '0');
        i++;
        any_digit = 1;
    }
    if (s[i] == '.')
    {
        i++;
        while (s[i] >= '0' && s[i] <= '9')
        {
            frac = frac * 10.0 + (double)(s[i] - '0');
            div *= 10.0;
            i++;
            any_digit = 1;
        }
    }
    if (!any_digit)
    {
        return false; /* 空串 / 纯符号 / 非数字 */
    }
    while (s[i] == ' ')
    {
        i++;
    }
    if (s[i] != '\0')
    {
        return false; /* 尾部残留非法字符（如科学计数法 e、单位等） */
    }
    *out = (float)(sign * (value + frac / div));
    return true;
}
