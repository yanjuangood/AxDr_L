/**
  ******************************************************************************
  * @file    foc_protect.c
  * @brief   保护模块 —— 过流 / 过压 / 欠压 / 过温 + 电流零偏自检
  *
  *  ⚠️ 这块板没有硬件过流保护!
  *     驱动芯片 FD6288Q 的 ITRIP / FAULT 引脚是真正的 NC (原理图上是空脚),
  *     母线也没有采样电阻, 所以 「过流保护 = 本文件」。软件写错 = 炸管。
  *
  *  保护逻辑:
  *    1. 每个阈值都有独立计数器做去抖 (连续 N 次超限才跳闸), 门限和次数都在
  *       pmsm_protect_init() 里设置
  *    2. 过流额外有一条「硬限」: 超过 oc_value * PROT_OC_HARD_RATIO 立即跳闸,
  *       不去抖 —— 半桥直通这类事故等不了去抖窗口
  *    3. 任何故障位被置起 -> pmsm_fault_stop_mode(): 立刻关 PWM + 清指令,
  *       并把 ctrl_bit 打到 reset。故障是闭锁的, 只能靠 pmsm_reset() 清除
  ******************************************************************************
  */
#include "common.h"
#include "ntc.h"

/* ===========================================================================
 *  阈值 —— 按你的电机和电源改这里
 * =========================================================================== */

/* 母线电压 (V)。
 * 分压 R81(10K)+R82(10K)+R83(1K), k = 21, 量程约 69V。
 * 本机电源 12V: 欠压 8V (留足电池跌落到 9~10V 的余量) */
#define PROT_UV_VALUE       8.0f

/* 过压 (V)。12V 系统取 18V (1.5 倍), 比原来的 30V 收得紧 */
#define PROT_OV_VALUE       18.0f

/* 过流 (A)。三相电流取绝对值后的最大值。
 * 2804: 额定 0.5A, 堵转 1.8A。门限取 1.5A —— 高于额定三倍, 低于堵转 */
#define PROT_OC_VALUE       1.5f

/* 过流硬限倍数: 超过 oc_value * 该值立刻跳闸, 不去抖 */
#define PROT_OC_HARD_RATIO  2.0f

/* 去抖计数 (ISR 20kHz -> 1 计数 = 50us) */
#define PROT_UV_CNT         200u    /* 10 ms  */
#define PROT_OV_CNT         200u    /* 10 ms  */
#define PROT_OC_CNT         5u      /* 250 us */
#define PROT_OT_CNT         2000u   /* 100 ms */
#define PROT_OMT_CNT        2000u   /* 100 ms */

/* MOS 过温 (°C) —— 与 ntc.h 的 NTC_TMOS_FAULT 保持一致 */
#define PROT_OT_VALUE       100.0f

/* 绕组过温 (°C) —— 与 ntc.h 的 NTC_TCOIL_FAULT 保持一致 */
#define PROT_OMT_VALUE      120.0f

/* 电流零偏自检窗口 (ADC 计数)。
 * RS624 差分放大器的输出偏置在 AVCC/2, 正常 ADC 该读到约 2048。
 * 读到接近 0 或满量程 -> VREF 没建立起来。
 * ⚠️ 板上 U14 (TLV9001IDCKR) 就是给 VREF 做缓冲的。没焊的话这里必然失败,
 *    四路电流全部无效 —— 此时绝不能让电流环跑起来。 */
#define PROT_IOFF_MIN       1500.0f
#define PROT_IOFF_MAX       2600.0f

/* 上电保护屏蔽时间 (ISR 计数, 20kHz -> 1 计数 = 50us)。
 * 必须留这段时间: 上电到 ADC 正常出数之间, JDR 寄存器是 0 或满量程,
 * 直接判故障会误报欠压 + 过压 —— 而故障是闭锁的, 一报就再也起不来。
 * 实测: 不加这段, 上电必然误报 ov_volt (读满量程 4095 -> 算出 69V)。 */
#define PROT_ARM_DELAY      4000u   /* 200 ms */

/* ===========================================================================
 *  降级模式过流兜底 (U14 未焊时)
 *
 *  U14 不在 -> VREF = 0 -> RS624 输出变成单极性:
 *      ADC 计数 = 4096 * 20 * I * Rshunt / 3.3 = 24.8 * I   (只对正电流)
 *  实测空闲读数: ia=0, ib=0, ic=20, 和这个推算吻合。
 *
 *  正常路径的零偏补偿 (i = (adc - 2048) * i_ratio) 在这种情况下没法用,
 *  但原始计数仍然单调反映电流大小, 所以直接拿它做一道粗保护。
 *
 *  阈值按 2804 定 (额定 0.5A / 堵转 1.8A):
 *      运行电流约 0.35~0.46A  -> 18(偏置) + 9~11 = 27~29 counts
 *      堵转电流 1.8A          -> 18 + 45        = 63 counts
 *      门限取 70 counts ≈ 2.1A: 高于堵转, 高于运行值 2.4 倍, 不会误报
 *  实测空闲读数 0/0/19, 所以余量充足。
 *
 *  去抖 3 次 (150us): 滤掉 PWM 开关尖峰, 同时保留足够的响应速度。
 *  ⚠️ U14 焊上之后 ioff_err 会清掉, 走上面的正常路径, 这段自动失效。
 * =========================================================================== */
#define PROT_RAW_OC_LIMIT   70u
#define PROT_RAW_OC_CNT     3u

/* ===========================================================================
 *  保护使能状态
 *
 *  ⚠️ 这两个静态变量是保护逻辑的时序基础, 别改成局部变量。
 *
 *  为什么不能只靠固定延时:
 *    pmsm_init() -> foc_get_curr_off() 要跑整 1 秒 (1000 次 1ms 延时) 采零偏,
 *    而这段时间里 ADC 中断早就在跑了。如果保护从第一个中断就开始计时, 屏蔽期
 *    (200ms) 会在零偏采集途中就结束 —— 此时 ia_off 还是 0, 而 ioff_err 还没
 *    置起来, 过流判断会把 (adc-0)*0.0403 算成几十安培, 直接误跳闸并闭锁。
 *
 *  正确顺序: pmsm_protect_init() 全部做完 (阈值 + 零偏自检) 才把
 *  s_prot_ready 置 1, 从那一刻起再数屏蔽期。
 * =========================================================================== */
static uint8_t  s_prot_ready = 0u;   /* 1 = pmsm_protect_init() 已完成 */
static uint32_t s_arm_cnt    = 0u;   /* 屏蔽期计数 */

/**
***********************************************************************
* @brief:      pmsm_protect_init(void)
* @param[in]:  void
* @retval:     void
* @details:    保护阈值初始化 + 电流零偏自检
* @note        必须在 foc_get_curr_off() 之后调用, 自检要用到零偏值
***********************************************************************
**/
void pmsm_protect_init(void)
{
    /* ---- 阈值 ---- */
    pm.protect.uv_value  = PROT_UV_VALUE;
    pm.protect.ov_value  = PROT_OV_VALUE;
    pm.protect.oc_value  = PROT_OC_VALUE;
    pm.protect.ot_value  = PROT_OT_VALUE;
    pm.protect.omt_value = PROT_OMT_VALUE;

    /* ---- 去抖次数 ---- */
    pm.protect.uv_cnt_value  = PROT_UV_CNT;
    pm.protect.ov_cnt_value  = PROT_OV_CNT;
    pm.protect.oc_cnt_value  = PROT_OC_CNT;
    pm.protect.ot_cnt_value  = PROT_OT_CNT;
    pm.protect.omt_cnt_value = PROT_OMT_CNT;

    /* ---- 计数器清零 ---- */
    pm.protect.uv_cnt  = 0u;
    pm.protect.ov_cnt  = 0u;
    pm.protect.oc_cnt  = 0u;
    pm.protect.ot_cnt  = 0u;
    pm.protect.omt_cnt = 0u;

    /* ---- 电流零偏自检 ----
     * 零偏不在合理窗口内, 说明 VREF 没建立 -> 电流采样无效。
     * 置 ioff_err 会挡住电流闭环 (见 foc_loop.c), 但不影响开环 V/f。
     */
    if ((pm.adc.ia_off < PROT_IOFF_MIN) || (pm.adc.ia_off > PROT_IOFF_MAX) ||
        (pm.adc.ib_off < PROT_IOFF_MIN) || (pm.adc.ib_off > PROT_IOFF_MAX) ||
        (pm.adc.ic_off < PROT_IOFF_MIN) || (pm.adc.ic_off > PROT_IOFF_MAX))
    {
        pm.fault.bit.ioff_err = 1u;
    }

    /* ---- 最后一步才放行保护 ---- */
    s_arm_cnt = 0u;
    s_prot_ready = 1u;
}

/**
***********************************************************************
* @brief:      pmsm_fault_check(pmsm_t* pm)
* @param[in]:  pm  指向 PMSM 控制结构体的指针
* @retval:     void
* @details:    故障检测 —— 母线电压、三相过流、MOS/绕组过温。
*              所有故障都是闭锁的, 由 pmsm_reset() 统一清除
* @note        在 20kHz 的 ADC 中断里调用, 必须够快 (只有比较和计数)
***********************************************************************
**/
void pmsm_fault_check(pmsm_t* pm)
{
    float abs_a, abs_b, abs_c, i_peak;

    /* ==================== 使能 + 上电屏蔽期 ====================
     * pmsm_protect_init() 还没跑完 -> 阈值全是 0, 判什么都是误报, 直接返回。
     * 这一步是必须的: 零偏采集要 1 秒, 期间 ADC 中断已经在跑, 如果这时就
     * 开始判过流, ia_off 还是 0, 会把正常读数算成几十安培。 */
    if (s_prot_ready == 0u)
    {
        return;
    }

    if (s_arm_cnt < PROT_ARM_DELAY)
    {
        s_arm_cnt++;

        /* 屏蔽期内只清计数器, 不做任何判断 */
        pm->protect.uv_cnt  = 0u;
        pm->protect.ov_cnt  = 0u;
        pm->protect.oc_cnt  = 0u;
        pm->protect.ot_cnt  = 0u;
        pm->protect.omt_cnt = 0u;

        return;
    }

    /* ==================== 母线欠压 ==================== */
    if (pm->foc.vbus < pm->protect.uv_value)
    {
        if (pm->protect.uv_cnt < pm->protect.uv_cnt_value)
        {
            pm->protect.uv_cnt++;
        }
        else
        {
            pm->fault.bit.un_volt = 1u;
        }
    }
    else
    {
        pm->protect.uv_cnt = 0u;
    }

    /* ==================== 母线过压 ==================== */
    if (pm->foc.vbus > pm->protect.ov_value)
    {
        if (pm->protect.ov_cnt < pm->protect.ov_cnt_value)
        {
            pm->protect.ov_cnt++;
        }
        else
        {
            pm->fault.bit.ov_volt = 1u;
        }
    }
    else
    {
        pm->protect.ov_cnt = 0u;
    }

    /* ==================== 过流 ====================
     * ⚠️ 电流通道无效 (ioff_err) 时跳过这一段。
     *    零偏不是 2048 而是 0 时, i = (adc - 0) * 0.0403, 任何正常读数都会
     *    被算成几十安培, 判了只会误报。读数本身就是假的, 判它没有意义。
     *    此时靠 ioff_err 挡住电流闭环 (见 foc_loop.c), 开环 V/f 还能跑 ——
     *    但要注意: 这块板没有硬件过流保护, 所以 U14 焊上之前是真没有过流保护。 */
    if (pm->fault.bit.ioff_err == 0u)
    {
        abs_a = ABS(pm->foc.i_a);
        abs_b = ABS(pm->foc.i_b);
        abs_c = ABS(pm->foc.i_c);
        i_peak = max(max(abs_a, abs_b), abs_c);

        if (i_peak > (pm->protect.oc_value * PROT_OC_HARD_RATIO))
        {
            /* 硬限: 立刻跳闸, 不去抖 */
            pm->fault.bit.ov_curr = 1u;
        }
        else if (i_peak > pm->protect.oc_value)
        {
            if (pm->protect.oc_cnt < pm->protect.oc_cnt_value)
            {
                pm->protect.oc_cnt++;
            }
            else
            {
                pm->fault.bit.ov_curr = 1u;
            }
        }
        else
        {
            pm->protect.oc_cnt = 0u;
        }
    }
    else
    {
        /* 降级模式兜底 (U14 未焊, VREF=0)。详见文件顶部 PROT_RAW_OC_LIMIT 的说明。
         * 用原始 ADC 计数而不是换算后的电流值 —— 靠的就是计数仍然单调 */
        static uint8_t raw_oc_cnt = 0u;

        if ((pm->adc.ia > PROT_RAW_OC_LIMIT) ||
            (pm->adc.ib > PROT_RAW_OC_LIMIT) ||
            (pm->adc.ic > PROT_RAW_OC_LIMIT))
        {
            if (raw_oc_cnt < PROT_RAW_OC_CNT)
            {
                raw_oc_cnt++;
            }
            else
            {
                pm->fault.bit.ov_curr = 1u;
            }
        }
        else
        {
            raw_oc_cnt = 0u;
        }

        pm->protect.oc_cnt = 0u;
    }

    /* ==================== MOS 过温 ==================== */
    /* ntc_mos 由 ntc.c 在主循环 100Hz 更新。
     * valid == 0 表示传感器断线/短路 —— 不拿它当合法温度,
     * 否则断线会读成"很冷", 过温保护就永久失效了。 */
    if ((ntc_mos.valid != 0u) && (ntc_mos.temp > pm->protect.ot_value))
    {
        if (pm->protect.ot_cnt < pm->protect.ot_cnt_value)
        {
            pm->protect.ot_cnt++;
        }
        else
        {
            pm->fault.bit.ov_tmos = 1u;
        }
    }
    else
    {
        pm->protect.ot_cnt = 0u;
    }

    /* ==================== 绕组过温 ==================== */
    if ((ntc_coil.valid != 0u) && (ntc_coil.temp > pm->protect.omt_value))
    {
        if (pm->protect.omt_cnt < pm->protect.omt_cnt_value)
        {
            pm->protect.omt_cnt++;
        }
        else
        {
            pm->fault.bit.ov_tcoi = 1u;
        }
    }
    else
    {
        pm->protect.omt_cnt = 0u;
    }

    /* ==================== 故障动作 ==================== */
    /* 这里只处理「必须立刻停机」的硬故障。
     *
     * ⚠️ ioff_err 刻意不在这个列表里:
     *    它只说明电流采样通道不可用 (典型原因: 板上 U14 没焊, VREF 没建立),
     *    此时开环 V/f 依然可以正常工作。如果这里把 PWM 一起拉停, 就等于
     *    因为"测不了电流"而禁掉了整个驱动器。
     *    正确做法是让 ioff_err 只挡住电流闭环 —— 见 foc_loop.c 的 foc_curr()。
     */
    if ((pm->fault.bit.ov_curr != 0u) || (pm->fault.bit.un_volt != 0u) ||
        (pm->fault.bit.ov_volt != 0u) || (pm->fault.bit.ov_tmos != 0u) ||
        (pm->fault.bit.ov_tcoi != 0u) || (pm->fault.bit.enc_err != 0u))
    {
        pmsm_fault_stop_mode(pm);
    }
}

/**
***********************************************************************
* @brief:      pmsm_fault_stop_mode(pmsm_t* pm)
* @param[in]:  pm  指向 PMSM 控制结构体的指针
* @retval:     void
* @details:    故障停机 —— 立即关闭三相 PWM, 清掉所有指令, 把状态推到 reset
* @note        关 PWM 放在最前面。先清变量再关 PWM 会多出几个周期的输出
***********************************************************************
**/
void pmsm_fault_stop_mode(pmsm_t* pm)
{
    /* 1. 最先关输出 */
    foc_pwm_stop();

    /* 2. 再清指令和积分, 避免故障恢复瞬间带着旧积分冲出去 */
    pm->ctrl.wr_set = 0.0f;
    pm->ctrl.we_set = 0.0f;
    pm->ctrl.vd_set = 0.0f;
    pm->ctrl.vq_set = 0.0f;
    pm->ctrl.tor_set = 0.0f;

    pid_clear(&pm->id_pi);
    pid_clear(&pm->iq_pi);
    pid_clear(&pm->vq_pi);

    /* 3. 状态推到 reset, 让状态机保持停机 */
    pm->ctrl_bit = reset;
}

/**
***********************************************************************
* @brief:      pmsm_reset(pmsm_t* pm)
* @param[in]:  pm  指向 PMSM 控制结构体的指针
* @retval:     void
* @details:    故障复位 —— 清故障位和所有去抖计数器, 清 PID 状态, 回到 start
* @note        这是唯一能解除故障闭锁的入口。上电时 pm 被 memset 清零,
*              所以必须显式调一次, 不能指望默认值
***********************************************************************
**/
void pmsm_reset(pmsm_t* pm)
{
    /* 故障位清零 */
    pm->fault.all = 0u;

    /* 去抖计数器清零 (不清的话故障会立刻复发) */
    pm->protect.uv_cnt   = 0u;
    pm->protect.ov_cnt   = 0u;
    pm->protect.oc_cnt   = 0u;
    pm->protect.ot_cnt   = 0u;
    pm->protect.omt_cnt  = 0u;
    pm->protect.link_out_cnt = 0u;
    pm->protect.ov_speed_cnt = 0u;

    /* 控制器状态清零 */
    pid_clear(&pm->id_pi);
    pid_clear(&pm->iq_pi);
    pid_clear(&pm->vq_pi);

    pm->ctrl_bit = start;
}

/**
***********************************************************************
* @brief:      temp_calc(void)
* @param[in]:  void
* @retval:     void
* @details:    温度采集入口 —— 调用 ntc.c 完成采样/换算/滤波/过温判断,
*              并把结果同步到显示结构
* @note        建议在主循环里 100Hz 调用
***********************************************************************
**/
void temp_calc(void)
{
    /* 采样 + B 值换算 + 一阶滤波 + 过温标志 (实现见 ntc.c) */
    ntc_update();

    /* 同步到显示结构, 保持与原工程接口一致 */
    pm.display.Tmos  = pm.foc.Tmos;
    pm.display.Tcoil = pm.foc.Tcoil;
}
