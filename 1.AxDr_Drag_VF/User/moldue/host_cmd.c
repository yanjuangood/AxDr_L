/**
***********************************************************************
* @file    host_cmd.c
* @brief   上位机命令解析 (USB CDC, 文本协议)
*
*  设计要点:
*   1. 表驱动 —— 加一个参数只需要在 s_params[] 里加一行, 解析和执行都不用改
*   2. 中断只负责收字节 (host_cmd_feed), 解析和执行都在主循环 (host_cmd_poll),
*      不在中断里做 snprintf / 浮点运算
*   3. 回复走独立缓冲 + 重试 —— CDC 一直在发 100Hz 的 VOFA 数据流,
*      CDC_Transmit_FS 经常返回 BUSY, 直接丢弃会丢回复
***********************************************************************
**/
#include "host_cmd.h"
#include "common.h"
#include "usbd_cdc_if.h"        /* CDC_Transmit_FS */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

/* ======================= 可调缓冲区大小 ======================= */
#define HC_RX_SIZE      256u    /* 接收环形缓冲 */
#define HC_LINE_SIZE    96u     /* 单行命令最大长度 */

/* 回复缓冲。HELP 要列出全部参数 (~900 字节), GET? 也要 350 字节左右,
 * 所以 512 是不够的 —— 之前就是因为截断在 512, 结尾的 '>>' 发不出去,
 * 上位机永远配不上 << >>, 表现成"收到几百帧但 0 个参数"。 */
#define HC_TX_SIZE      2048u

/* ======================= 运行时可调参数 ======================= */

/* 编码器计数方向。+1 / -1, 见 sensory_pos_calc() 的说明。
 * 定义成运行时变量而不是宏, 这样上位机能改。 */
float   host_enc_dir = -1.0f;

/* 电角度零点标定触发标志: 上位机发 CAL 时置 1, 主循环看到后跑一次标定 */
volatile uint8_t host_cal_request = 0u;

/* ======================= 接收环形缓冲 ======================= */
static volatile uint8_t  s_rx[HC_RX_SIZE];
static volatile uint16_t s_rx_head = 0u;
static volatile uint16_t s_rx_tail = 0u;

/* ======================= 回复缓冲 (单生产者单消费者) ======================= */
static char     s_tx[HC_TX_SIZE];
static uint16_t s_tx_len = 0u;      /* 已填字节数 */
static uint16_t s_tx_sent = 0u;     /* 已发出字节数 */

/* ======================= 行缓冲 ======================= */
static char     s_line[HC_LINE_SIZE];
static uint16_t s_line_len = 0u;

/* ======================= 参数表 ======================= */

typedef enum
{
    PT_F,       /* float  */
    PT_I,       /* int32  */
    PT_U        /* uint32 */
} hc_type_e;

typedef struct
{
    const char *name;       /* 命令里用的名字 */
    hc_type_e   type;
    void       *ptr;        /* 指向真正的变量 */
    float       min;
    float       max;
    const char *desc;       /* 给 HELP 用 */
} hc_param_t;

static const hc_param_t s_params[] =
{
    /* ---- 运行控制 ---- */
    { "mode",   PT_I, &pm.foc.mode,      0.0f,      4.0f,  "0=停机 1=电压(V/f) 2=电流 3=速度 4=位置" },
    { "spd",    PT_F, &pm.ctrl.wr_set,  -200.0f,  200.0f,  "速度给定 (rad/s, 机械)" },
    { "id",     PT_F, &pm.ctrl.id_set,   -2.0f,     2.0f,  "d 轴电流给定 (A), 表贴式给 0" },
    { "iq",     PT_F, &pm.ctrl.iq_set,   -2.0f,     2.0f,  "q 轴电流给定 (A), 对应转矩" },
    { "vq",     PT_F, &pm.ctrl.vq_set,   -6.0f,     6.0f,  "q 轴电压给定 (V), 仅 V/f 模式用" },
    { "vd",     PT_F, &pm.ctrl.vd_set,   -6.0f,     6.0f,  "d 轴电压给定 (V), 仅 V/f 模式用" },

    /* ---- 电流环 ---- */
    { "ibw",    PT_F, &pm.para.ibw,      10.0f,  3000.0f,  "电流环带宽 (Hz)" },
    { "iqmax",  PT_F, &pm.ctrl.pmax_iq,   0.0f,     2.0f,  "正向电流限幅 (A)" },
    { "iqmin",  PT_F, &pm.ctrl.nmax_iq,  -2.0f,     0.0f,  "反向电流限幅 (A)" },

    /* ---- 电机参数 ---- */
    { "pn",     PT_F, &pm.para.pn,        1.0f,    30.0f,  "极对数" },
    { "rs",     PT_F, &pm.para.Rs,        0.01f,  100.0f,  "相电阻 (ohm)" },
    { "ld",     PT_F, &pm.para.Ld,     1e-5f,      0.1f,   "d 轴电感 (H)" },
    { "lq",     PT_F, &pm.para.Lq,     1e-5f,      0.1f,   "q 轴电感 (H)" },
    { "flux",   PT_F, &pm.para.flux,    1e-4f,      0.1f,  "磁链 (Wb)" },
    { "js",     PT_F, &pm.para.Js,      1e-8f,      1e-2f, "转动惯量 (kg*m^2)" },
    { "b",      PT_F, &pm.para.B,       0.0f,       1.0f,  "粘滞阻尼" },

    /* ---- 编码器 / 电角度 ---- */
    { "eoff",   PT_F, &pm.para.e_off,   0.0f,       6.2832f, "电角度零点 (rad)" },
    { "encdir", PT_F, &host_enc_dir,   -1.0f,       1.0f,  "编码器方向 +1/-1" },
};

#define HC_NPARAM  (sizeof(s_params) / sizeof(s_params[0]))

/* ======================= 回复输出 ======================= */

/* 把一段文本追加到待发缓冲。满了就丢掉, 不阻塞 */
static void hc_puts(const char *s)
{
    while (*s != '\0')
    {
        if (s_tx_len >= HC_TX_SIZE)
        {
            return;
        }
        s_tx[s_tx_len++] = *s++;
    }
}

/* 追加一行, 自动补 \r\n */
static void hc_line(const char *s)
{
    hc_puts(s);
    hc_puts("\r\n");
}

static void hc_printf(const char *fmt, ...)
{
    char tmp[96];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    hc_puts(tmp);
}

/* ======================= 参数查找与读写 ======================= */

static const hc_param_t *hc_find(const char *name)
{
    uint32_t i;

    for (i = 0u; i < HC_NPARAM; i++)
    {
        if (strcmp(s_params[i].name, name) == 0)
        {
            return &s_params[i];
        }
    }
    return NULL;
}

static float hc_get(const hc_param_t *p)
{
    switch (p->type)
    {
    case PT_F:  return *(float *)p->ptr;
    case PT_I:  return (float)(*(int32_t *)p->ptr);
    default:    return (float)(*(uint32_t *)p->ptr);
    }
}

static void hc_set(const hc_param_t *p, float v)
{
    /* 限幅, 防止上位机手滑 */
    if (v < p->min) { v = p->min; }
    if (v > p->max) { v = p->max; }

    switch (p->type)
    {
    case PT_F:  *(float *)p->ptr    = v;                          break;
    case PT_I:  *(int32_t *)p->ptr  = (int32_t)v;                 break;
    default:    *(uint32_t *)p->ptr = (uint32_t)v;                break;
    }
}

/* ======================= 命令处理 ======================= */

/* 把整张参数表打出来 */
static void hc_dump_all(void)
{
    uint32_t i;

    hc_line("# BEGIN");
    for (i = 0u; i < HC_NPARAM; i++)
    {
        const hc_param_t *p = &s_params[i];

        if (p->type == PT_F)
        {
            hc_printf("%s=%.6g\r\n", p->name, (double)hc_get(p));
        }
        else
        {
            hc_printf("%s=%d\r\n", p->name, (int)hc_get(p));
        }
    }
    /* 实时量也一起带上, 方便上位机一次刷新 */
    hc_printf("vbus=%.3f\r\n",  (double)pm.foc.vbus);
    hc_printf("ia=%.4f\r\n",    (double)pm.foc.i_a);
    hc_printf("ib=%.4f\r\n",    (double)pm.foc.i_b);
    hc_printf("ic=%.4f\r\n",    (double)pm.foc.i_c);
    hc_printf("id_mea=%.4f\r\n",(double)pm.foc.i_d);
    hc_printf("iq_mea=%.4f\r\n",(double)pm.foc.i_q);
    hc_printf("wr=%.3f\r\n",    (double)pm.foc.wr);
    hc_printf("pe=%.4f\r\n",    (double)pm.foc.p_e);
    hc_printf("fault=%u\r\n",   (unsigned)pm.fault.all);
    hc_printf("ctrl=%u\r\n",    (unsigned)pm.ctrl_bit);
    hc_line("# END");
}

static void hc_help(void)
{
    uint32_t i;

    hc_line("# 参数表: 名字 类型 范围 说明");
    for (i = 0u; i < HC_NPARAM; i++)
    {
        const hc_param_t *p = &s_params[i];
        hc_printf("#   %-8s %-4s [%.4g, %.4g]  %s\r\n",
                  p->name,
                  (p->type == PT_F) ? "f" : "i",
                  (double)p->min, (double)p->max,
                  p->desc);
    }
    hc_line("# 命令: GET? / SET <名> <值> / <名> <值> / <名>?");
    hc_line("#       START / STOP / RST / CAL / HELP");
}

static void hc_exec_impl(char *line)
{
    char *cmd;
    char *arg;
    const hc_param_t *p;

    /* 去掉前后空白 */
    while ((*line == ' ') || (*line == '\t')) { line++; }
    if (*line == '\0') { return; }

    cmd = line;
    arg = strchr(line, ' ');
    if (arg != NULL)
    {
        *arg++ = '\0';
        while ((*arg == ' ') || (*arg == '\t')) { arg++; }
    }

    /* 全大写命令先处理 */
    if (strcmp(cmd, "GET?") == 0) { hc_dump_all(); return; }
    if (strcmp(cmd, "HELP") == 0) { hc_help();     return; }
    if (strcmp(cmd, "CAL")  == 0)
    {
        host_cal_request = 1u;
        hc_line("OK CAL started");
        return;
    }
    if (strcmp(cmd, "START") == 0)
    {
        if (pm.ctrl_bit == reset) { pm.ctrl_bit = start; }
        hc_line("OK START");
        return;
    }
    if (strcmp(cmd, "STOP") == 0)
    {
        pm.ctrl_bit = reset;
        hc_line("OK STOP");
        return;
    }
    if (strcmp(cmd, "RST") == 0)
    {
        pmsm_reset(&pm);
        hc_line("OK RST");
        return;
    }

    /* SET <名> <值> 形式 */
    if (strcmp(cmd, "SET") == 0)
    {
        if (arg == NULL) { hc_line("ERR SET needs name"); return; }
        cmd = arg;
        arg = strchr(arg, ' ');
        if (arg != NULL)
        {
            *arg++ = '\0';
            while ((*arg == ' ') || (*arg == '\t')) { arg++; }
        }
    }

    /* <名>? 回读单个 */
    {
        size_t n = strlen(cmd);
        if ((n > 1u) && (cmd[n - 1u] == '?'))
        {
            cmd[n - 1u] = '\0';
            p = hc_find(cmd);
            if (p == NULL)
            {
                hc_printf("ERR unknown %s\r\n", cmd);
            }
            else if (p->type == PT_F)
            {
                hc_printf("%s=%.6g\r\n", p->name, (double)hc_get(p));
            }
            else
            {
                hc_printf("%s=%d\r\n", p->name, (int)hc_get(p));
            }
            return;
        }
    }

    p = hc_find(cmd);
    if (p == NULL)
    {
        hc_printf("ERR unknown %s\r\n", cmd);
        return;
    }

    /* 没有值 -> 回读 */
    if (arg == NULL)
    {
        if (p->type == PT_F)
        {
            hc_printf("%s=%.6g\r\n", p->name, (double)hc_get(p));
        }
        else
        {
            hc_printf("%s=%d\r\n", p->name, (int)hc_get(p));
        }
        return;
    }

    /* 有值 -> 写入 */
    {
        float v = (float)atof(arg);
        hc_set(p, v);
        if (p->type == PT_F)
        {
            hc_printf("OK %s=%.6g\r\n", p->name, (double)hc_get(p));
        }
        else
        {
            hc_printf("OK %s=%d\r\n", p->name, (int)hc_get(p));
        }
    }
}

/* ======================= 对外接口 ======================= */

/* 上位机发的文本回复和 100Hz 的二进制数据流共用同一个 USB CDC 口,
 * 所以回复必须加标记 << ... >>, PC 端才能把两者干净地分开:
 *      标记外面  = JustFloat 二进制帧
 *      标记里面  = ASCII 文本
 * 不做标记的话 PC 只能猜, 一旦文本里出现 00 00 80 7F 就彻底乱了 */
static void hc_exec(char *line)
{
    hc_puts("<<");
    hc_exec_impl(line);

    /* 万一正文把缓冲塞满了, 也不能把结尾标记挤掉 —— 挤掉的话上位机
     * 就一直等不到 '>>', 整段回复都作废。这里砍掉正文尾部保住标记。 */
    if (s_tx_len > (HC_TX_SIZE - 2u))
    {
        s_tx_len = HC_TX_SIZE - 2u;
    }
    hc_puts(">>");
}

/* 中断上下文: 只往环形缓冲塞字节 */
void host_cmd_feed(const uint8_t *buf, uint32_t len)
{
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        uint16_t next = (uint16_t)((s_rx_head + 1u) % HC_RX_SIZE);

        if (next == s_rx_tail)
        {
            break;              /* 满, 丢弃 */
        }
        s_rx[s_rx_head] = buf[i];
        s_rx_head = next;
    }
}

/* 主循环: 取字节攒成行, 遇到 \n 就执行 */
void host_cmd_poll(void)
{
    while (s_rx_tail != s_rx_head)
    {
        uint8_t c = s_rx[s_rx_tail];

        s_rx_tail = (uint16_t)((s_rx_tail + 1u) % HC_RX_SIZE);

        if ((c == '\n') || (c == '\r'))
        {
            if (s_line_len > 0u)
            {
                s_line[s_line_len] = '\0';
                hc_exec(s_line);
                s_line_len = 0u;
            }
        }
        else if (s_line_len < (HC_LINE_SIZE - 1u))
        {
            s_line[s_line_len++] = (char)c;
        }
        else
        {
            s_line_len = 0u;    /* 行太长, 丢掉重新来 */
        }
    }
}

/* 主循环: 把待发缓冲里的内容尽量发出去 */
void host_cmd_tx_task(void)
{
    while (s_tx_sent < s_tx_len)
    {
        uint16_t chunk = (uint16_t)(s_tx_len - s_tx_sent);

        if (chunk > 64u) { chunk = 64u; }

        if (CDC_Transmit_FS((uint8_t *)&s_tx[s_tx_sent], chunk) != USBD_OK)
        {
            return;             /* CDC 忙, 下次再试 */
        }
        s_tx_sent += chunk;
    }

    /* 发完了, 清空缓冲准备下一批 */
    s_tx_len  = 0u;
    s_tx_sent = 0u;
}
