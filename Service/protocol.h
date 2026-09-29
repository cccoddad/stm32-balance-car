#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file protocol.h
 * @brief 串口遥控/调参协议解析状态机，纯 C 实现，零硬件依赖。
 *
 * 协议格式（带帧头帧尾，支持在线调参）：
 *   @MV,U#        遥控前后：U 前进 / D 后退 / S 停止
 *   @TR,L#        遥控转向：L 左 / R 右 / S 停止
 *   @PID,BKP,-720.0#   在线调参：参数名 + 数值
 *   @ST#          状态查询：角度/电压/周期统计/栈水位/参数快照
 *
 * 参数名约定：BKP 直立 kp、BKD 直立 kd、BANG 目标平衡角、
 *             VKP 速度 kp、VKI 速度 ki、TKP 转向 kp
 *
 * 兼容性：不带帧头的单字符 U/D/L/R/S 仍按遥控处理（课程配套蓝牙助手
 * 发的就是裸字符），旧上位机无需修改即可继续使用。
 *
 * 健壮性要求（已通过 10 万字节模糊测试，见 Host/tests/test_protocol.c）：
 * 二进制垃圾、半包、超长帧、非法字符一律丢弃归零，任何输入都不越界、不崩溃。
 */

/* 单帧最大长度（不含帧头 @ 与帧尾 #）。 */
#define PROTO_FRAME_MAX (32u)
/* 单个字段最大长度（含结束符）。 */
#define PROTO_ARG_MAX    (16u)

typedef enum {
    PROTO_NONE = 0, /* 未构成完整有效指令 */
    PROTO_MOVE,     /* 遥控前后：arg1[0] 为 U/D/S */
    PROTO_TURN,     /* 遥控转向：arg1[0] 为 L/R/S */
    PROTO_PID,      /* 在线调参：arg1 参数名，arg2 数值字符串 */
    PROTO_STATUS    /* 状态查询 @ST#：不带参数，输出由上层延迟到任务上下文打印 */
} proto_cmd_t;

/* 一条解析完成的指令。 */
typedef struct {
    proto_cmd_t cmd;
    char arg1[PROTO_ARG_MAX];
    char arg2[PROTO_ARG_MAX];
} proto_msg_t;

/* 解析状态机实例。 */
typedef struct {
    char buf[PROTO_FRAME_MAX]; /* 帧体缓冲（@ 与 # 之间内容） */
    uint8_t len;               /* 当前帧体长度 */
    uint8_t in_frame;          /* 1 = 已见到 @，正在收帧 */
} proto_parser_t;

/* 初始化解析状态机（清空缓冲与收帧状态）。 */
void Proto_Init(proto_parser_t *p);

/**
 * @brief 逐字节喂入串口数据，驱动状态机。
 * @param p 解析状态机实例。
 * @param byte 新收到的字节。
 * @param msg 解析出的完整指令（仅当返回 true 时有效）。
 * @return true = 已解析出一条指令；false = 字节被消费但未构成指令。
 *
 * 任意输入序列下都不修改 msg 以外的内存，可直接用于模糊测试。
 */
bool Proto_Feed(proto_parser_t *p, uint8_t byte, proto_msg_t *msg);

/**
 * @brief 解析十进制浮点字符串（支持符号与小数点，不支持科学计数法）。
 * @param s 待解析字符串。
 * @param out 解析结果输出。
 * @return true 解析成功；false 输入非法（out 保持不变）。
 */
bool Proto_ParseFloat(const char *s, float *out);

#endif /* __PROTOCOL_H */
