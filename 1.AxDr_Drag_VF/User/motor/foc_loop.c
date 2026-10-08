/**
  ******************************************************************************
  * @file    foc_loop.c
  * @brief   闭环控制 —— 电流环 PI、速度环 PI、位置计算
  *
  *  串级结构:
  *      位置环 (5kHz)  ->  速度环 (10kHz)  ->  电流环 (20kHz)  ->  SVPWM
  *                                              ^^^^^^ 本文件的核心
  *
  *  电流环 (foc_cur_pi_calc / foc_curr):
  *      1. 三相电流 -> Clarke -> Park -> i_d / i_q
  *      2. i_d / i_q 各自一个 PI -> v_d / v_q
  *      3. v_d / v_q -> 逆 Park -> SVPWM
  *
  *  PI 参数用「带宽整定」算, 不靠试凑:
  *      被控对象近似为 1/(Ls*s + Rs)
  *      Kp = wc * L,   Ki = wc * R * Ts
  *      wc = 2*pi*ibw,  ibw 取 pm.para.ibw (默认 1000Hz)
  *  这样改电机参数 (Ld/Lq/Rs) 后增益会自动跟着变, 不用重调。
  *
  *  ⚠️ 前置条件: 板上 U14 (TLV9001, VREF 缓冲) 必须焊上。
  *     没焊的话四路电流全无效, pmsm_protect_init() 会置 ioff_err 把 PWM 拉停,
  *     电流环根本进不来 —— 这是刻意的保护, 不是 bug。
  ******************************************************************************
  */
#include "common.h"
#include "mt6701.h"

/* 1 = 用编码器电角度做电压控制，核对标定得到的 e_off */
volatile uint8_t enc_volt_en = 0u;

/* 电流给定的对称限幅 (A)。正常运行的最大电流, 保护阈值在 foc_protect.c */
#define CUR_IQ_LIMIT        2.0f

/* d 轴电流给定: 表贴式永磁电机用 id=0 控制 (最大转矩/电流) */
#define CUR_ID_SET          0.0f

/* 速度环带宽 (Hz), 一般取电流环带宽的 1/5 ~ 1/10 */
#define SPD_LOOP_BW_HZ      100.0f

/**
***********************************************************************
* @brief:      sensory_pos_calc(pmsm_t* pm)
* @param[in]:  pm  指向 PMSM 控制结构体的指针
* @retval:     void
* @details:    有传感位置计算 —— 把 MT6701 的机械角换算成电角度, 写入 pm->foc.p_e
* @note        foc_para_calc() 直接拿 p_e 去算电角速度, 而 p_e 原来从来没被赋值过,
*              所以这一步是闭环的前提。角度不对电机会乱转甚至堵转发热
***********************************************************************
**/
/* 编码器计数方向相对 FOC 相序的方向。
 *
 *   +1 : 转子顺着相序正方向转时, mt6701.rad 增大
 *   -1 : 反过来
 *
 *  怎么判断: 如果方向反了, 电流环就成了正反馈 ——
 *            转子动一点, 命令角度往反方向跑, 转矩跟着翻转,
 *            结果是原地来回颤、净位移为零, 而且电流越大颤得越凶。
 *            2026-10 首次跑真 FOC 时就是这个症状 (iq=0.2A 时 ce_wr 在
 *            -13 ~ +26 rad/s 之间乱跳, 但转子只动 0.95 度), 改成 -1 后正常。
 *
 *  这个值现在是运行时变量 host_enc_dir (定义在 host_cmd.c),
 *  上位机可以改, 方便换电机/换编码器安装方向时不用重新编译。
 *
 *  ⚠️ 改这个值之后 e_off 必须重新标定 (CALIB_MODE / 上位机发 CAL),
 *     因为标定公式 e_off = CALIB_ANG - rad*pn 依赖于同一个方向约定。
 *     换方向时可以手算: e_off_new = CALIB_ANG + rad*pn */
void sensory_pos_calc(pmsm_t* pm)
{
    /* 编码器 CRC 不过 -> 角度不可信, 置故障停机。
     * 拿着错角度继续跑电流环比不跑危险得多 */
    if (mt6701.crc_ok == 0u)
    {
        pm->fault.bit.enc_err = 1u;
        return;
    }

    /* 机械角 (rad, 0~2pi) * 极对数 + 电角度零点偏移 = 电角度
     * 用 mt6701.rad 而不是 total_rad: rad 被限制在 0~2pi, 浮点精度更好,
     * 累计圈数多了之后 total_rad 的有效位会被吃掉 */
    pm->foc.p_e = host_enc_dir * (mt6701.rad * pm->para.pn) + pm->para.e_off;

    /* 归一化到 [0, 2pi) */
    wrap_0_2pi(pm->foc.p_e);
}

/**
***********************************************************************
* @brief:      foc_cur_pi_tune(pmsm_t* pm)
* @param[in]:  pm  指向 PMSM 控制结构体的指针
* @retval:     void
* @details:    按带宽整定电流环 PI 增益: Kp = wc*L, Ki = wc*R*Ts
* @note        直接写 kp/ki, 不调用 pid_reset —— pid_reset 会清掉积分项,
*              在 20kHz 里每周期清一次积分等于没有积分
***********************************************************************
**/
static void foc_cur_pi_tune(pmsm_t* pm)
{
    float wc = M_2PI * pm->para.ibw;      /* 目标带宽 (rad/s) */

    /* d 轴用 Ld, q 轴用 Lq (凸极电机两者不同) */
    pm->id_pi.kp = wc * pm->para.Ld;
    pm->id_pi.ki = wc * pm->para.Rs * pm->period.foc_ts;

    pm->iq_pi.kp = wc * pm->para.Lq;
    pm->iq_pi.ki = wc * pm->para.Rs * pm->period.foc_ts;

    pm->id_pi.ts = pm->period.foc_ts;
    pm->iq_pi.ts = pm->period.foc_ts;
}

/**
***********************************************************************
* @brief:      foc_cur_pi_calc(pmsm_t* pm)
* @param[in]:  pm  指向 PMSM 控制结构体的指针
* @retval:     void
* @details:    d/q 轴电流 PI 计算, 输出 v_d / v_q
* @note        限幅跟随母线电压 (pm->foc.vs): 母线跌落时输出上限自动收紧,
*              由 SVPWM 的过调制区兜底, 不会算出一个物理上给不出的电压
***********************************************************************
**/
_RAM_FUNC void foc_cur_pi_calc(pmsm_t* pm)
{
    float vlim = pm->foc.vs;

    /* 限幅值每周期刷新 (vs 依赖实时母线电压) */
    pid_limit_init(&pm->id_pi, vlim, -vlim, vlim, -vlim);
    pid_limit_init(&pm->iq_pi, vlim, -vlim, vlim, -vlim);

    /* d/q 各自独立 PI, 误差 = 给定 - 反馈 */
    pm->foc.v_d = parallel_pid_ctrl(&pm->id_pi, pm->ctrl.id_set, pm->foc.i_d);
    pm->foc.v_q = parallel_pid_ctrl(&pm->iq_pi, pm->ctrl.iq_set, pm->foc.i_q);
}

/**
***********************************************************************
* @brief:      foc_curr(pmsm_t* pm, float id_set, float iq_set, float pos)
* @param[in]:  pm      指向 PMSM 控制结构体的指针
* @param[in]:  id_set  d 轴电流给定 (A), 表贴式电机给 0
* @param[in]:  iq_set  q 轴电流给定 (A), 对应转矩
* @param[in]:  pos     电角度 (rad)
* @retval:     void
* @details:    电流环一个完整周期: 坐标变换 -> PI -> 逆变换 -> SVPWM
***********************************************************************
**/
_RAM_FUNC void foc_curr(pmsm_t* pm, float id_set, float iq_set, float pos)
{
    /* ⚠️ 电流采样通道无效时拒绝工作。
     * 典型原因: 板上 U14 (TLV9001, VREF 缓冲) 没焊, RS624 的偏置电压没建立,
     * 四路电流全是假值。拿假电流做闭环 = 盲开环, 比不开环危险得多。
     * 这里把状态打到 reset 关掉输出并停机, 而不是继续算。
     * 注意: 这个位只挡电流环, 开环 V/f 不受影响 (见 foc_protect.c) */
    if (pm->fault.bit.ioff_err != 0u)
    {
        pm->ctrl_bit = reset;
        return;
    }

    pm->foc.mode = foc_curr_mode;

    /* ---- 1. 电流给定限幅 ----
     * 这里限的是「正常运行范围」, 不是保护阈值。
     * 真正的过流保护在 foc_protect.c, 两者职责不同 */
    pm->ctrl.id_set = sat1_datf(id_set, CUR_IQ_LIMIT, -CUR_IQ_LIMIT);
    pm->ctrl.iq_set = sat1_datf(iq_set, CUR_IQ_LIMIT, -CUR_IQ_LIMIT);

    /* ---- 2. 电角度 -> sin/cos ---- */
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    sin_cos_val(&pm->foc);

    /* ---- 3. 三相电流 -> alpha/beta -> d/q ---- */
    clarke_transform(&pm->foc);
    park_transform(&pm->foc);

    /* ---- 4. 电流 PI ---- */
    foc_cur_pi_tune(pm);
    foc_cur_pi_calc(pm);

    /* ---- 5. d/q 电压 -> alpha/beta -> SVPWM ---- */
    inverse_park(&pm->foc);
    if (svm(pm->foc.v_alph * pm->foc.inv_vbus,
            pm->foc.v_beta * pm->foc.inv_vbus,
            &pm->foc.dtc_a, &pm->foc.dtc_b, &pm->foc.dtc_c) == 0)
    {
        foc_pwm_run(pm);
    }
    /* svm 返回 -1 说明算出来的占空比超出 [0,1] (过调制/除零),
       此时不更新 CCR, 保持上一次输出, 等下一个周期自己收敛回来 */
}

/**
***********************************************************************
* @brief:      foc_spd_pi_calc(pmsm_t* pm)
* @param[in]:  pm  指向 PMSM 控制结构体的指针
* @retval:     void
* @details:    速度环 PI, 输出作为 q 轴电流给定 (串级控制的外环)
* @note        用 PDFF 形式 (微分作用在反馈上), 速度给定阶跃时不会有微分冲击
***********************************************************************
**/
void foc_spd_pi_calc(pmsm_t* pm)
{
    float wc_spd = M_2PI * SPD_LOOP_BW_HZ;
    float iq;

    /* 速度环增益整定: 转动惯量 Js 与阻尼 B。
     * 带宽取得比电流环低一档, 保证串级稳定 */
    pm->vq_pi.kp = wc_spd * pm->para.Js;
    pm->vq_pi.ki = wc_spd * pm->para.B * pm->period.spd_pid_ts;
    pm->vq_pi.ts = pm->period.spd_pid_ts;

    /* 积分限幅 = 电流给定上限, 输出限幅同理 —— 速度环的输出是电流 */
    pid_limit_init(&pm->vq_pi, CUR_IQ_LIMIT, -CUR_IQ_LIMIT, CUR_IQ_LIMIT, -CUR_IQ_LIMIT);

    iq = pdff_ctrl(&pm->vq_pi, pm->ctrl.wr_set, pm->foc.wr);

    /* 再用控制参数里的正反向限幅夹一次 (用户可以在上位机单独限制) */
    pm->ctrl.iq_set = sat1_datf(iq, pm->ctrl.pmax_iq, pm->ctrl.nmax_iq);
}

/**
***********************************************************************
* @brief:      force_curr_mode(pmsm_t* pm)
* @param[in]:  pm  指向 PMSM 控制结构体的指针
* @retval:     void
* @details:    电流闭环模式入口 —— 取电角度, 检查故障, 然后跑电流环
* @note        电角度取不到 (编码器异常) 时直接返回, 绝不拿错角度去跑闭环
***********************************************************************
**/
_RAM_FUNC void force_curr_mode(pmsm_t* pm)
{
    /* 1. 先从编码器算出电角度 (失败会置 enc_err) */
    sensory_pos_calc(pm);

    /* 2. 有任何故障就不往下走 —— 故障时 p_e 可能是上一周期的旧值 */
    if (pm->fault.all != 0u)
    {
        return;
    }

    /* 3. 跑电流环。给定来自 pm->ctrl.id_set / iq_set */
    foc_curr(pm, pm->ctrl.id_set, pm->ctrl.iq_set, pm->foc.p_e);
}

/**
  ******************************************************************************
  *  开环 V/f 测试 (drag mode)
  *
  *  用途: 在没有位置反馈、没有电流反馈的情况下让电机先转起来, 验证
  *        功率级 / 相序 / 接线 / 编码器安装方向。
  *
  *  ⚠️ 安全须知 (这块板 + 2804 的组合):
  *     - 本板没有任何硬件过流保护 (FD6288Q 的 ITRIP/FAULT 是 NC)
  *     - 软件过流也指望不上: U14 (TLV9001) 没焊时电流采样是死的,
  *       降级兜底读的也是同一个坏通道 —— 等于没有过流保护
  *     - 开环是恒压驱动, 电流完全由「电压 / 相电阻」决定, 没有任何反馈限制它:
  *           2804 实测线电阻 6.2Ω -> 相电阻 Rs = 3.1Ω
  *           vq = 0.048 * wr + 1.5
  *           启动 (wr=0):  1.50V / 3.1 = 0.48 A   约等于额定
  *           满载 (wr=15): 2.22V / 3.1 = 0.72 A   约 1.4 倍额定
  *           堵转 (转子不动): 反电动势为 0, 0.72A 全部变成热 -> 7 秒约 11W
  *       ⇒ 所以必须靠 OL_STALL_* 那条堵转保护, 见下面的说明
  *     - 想更保险就在母线上串一个 2Ω/10W 功率电阻, 或用带限流的可调电源
  *
  *  V/f 曲线怎么定的:
  *     相电压 = 反电动势 + 电阻压降
  *            = wr * pn * flux + I * Rs
  *     2804: pn*flux = 7 * 0.0062 = 0.0434 V/(rad/s) 是反电动势常数。
  *     斜率取 0.048 (略高于它, 保证速度上去后还有电流余量),
  *     再加 OL_V_MIN 这个偏置 —— 这台电机电阻大, 偏置那一项才是力矩的主要来源。
  ******************************************************************************
  */

/* V/f 斜率 (V per rad/s)。
 * 2804: pn*flux = 7*0.0062 = 0.0434 V/(rad/s) 是反电动势常数。
 * 取 0.048, 略高于它, 保证速度上去后还有电流余量。
 *
 * ⚠️ 这台电机电阻大 (实测相电阻 3.1Ω), 电压大部分被电阻吃掉, 所以 vq 里
 *    OL_V_MIN 那一项才是力矩的主要来源 (见下面)。 */
#define OL_VF_RATIO     0.048f

/* 启动最低电压 (V)。
 * 2804: 1.5V / 3.1Ω(实测相电阻) = 0.48A ≈ 额定电流
 *       -> 力矩 0.031 N*m ≈ 额定扭矩, 够克服齿槽转矩起转 */
#define OL_V_MIN        1.5f

/* 目标机械角速度 (rad/s)。15 rad/s ≈ 143 rpm */
#define OL_WR_TARGET    15.0f

/* 加速度 (rad/s^2)。6 大约 2.5 秒到目标速度, 起步更从容 */
#define OL_RAMP_RATE    6.0f

/* ===========================================================================
 *  堵转保护 (OL_STALL_*)
 *
 *  为什么必须有: 开环 V/f 是「恒压驱动」, 输出电压只跟 wr_set 走, 完全不看
 *  转子有没有真的转。转子跟不上的时候, 反电动势为 0, 全部电压都加在相电阻上:
 *
 *      I = vq / Rs          (2804: Rs=3.1Ω)
 *
 *  以本机为例, 满载 2.22V 时 I = 0.72A, 全部变成热 (7 秒约 11W)。
 *  51g 的小电机几十秒就能把漆包线烧了。
 *
 *  ⚠️ 这不是理论风险 —— 之前就因为这个烧烫过一次电机, 当时没有这条检查。
 *
 *  判据: 命令转速已上到 OL_STALL_WR_MIN 以上, 但编码器实测转速低于命令的
 *        OL_STALL_RATIO 倍, 且持续 OL_STALL_SEC 秒 -> 判定堵转, 立刻停机。
 *
 *  用 mt6701.total_rad 而不是 pm.foc.wr: 开环时不调用 sensory_pos_calc(),
 *  p_e 不更新, foc.wr 是死的。
 *
 *  设 OL_STALL_ENABLE 0 可以关掉 (比如编码器没装时想纯开环跑),
 *  但那样就必须盯着电机温度, 别跑久。
 * =========================================================================== */
#define OL_STALL_ENABLE     1       /* 1 = 启用堵转保护 */
#define OL_STALL_WR_MIN     3.0f    /* 命令转速超过这个才检查 (rad/s) */
#define OL_STALL_RATIO      0.15f   /* 实测 < 命令的 15% 判为堵转 */
#define OL_STALL_SEC        1.5f    /* 持续这么久才判, 避开起步瞬间 */

/* 匀速保持时间 (秒), 之后自动减速停机。
 * 60 秒便于用 VOFA+ 观察; 跑完自动停, 不会忘记断电导致堵转发烫 */
#define OL_HOLD_SEC     60.0f

/* 主循环调用周期 (秒)。main.c 里是 HAL_Delay(10) */
#define OL_TS           0.01f

/* 定位时间 (秒)。电压矢量钉在 0 度, 让转子先对到已知位置再开始加速。
 * 这段 wr=0, 堵转保护不会介入。
 * 原来 8 秒太长 (上电 5s 等待 + 8s 对准 = 13 秒才转起来), 3 秒足够定位。 */
#define OL_ALIGN_SEC    3.0f

/* 状态机 */
enum
{
    OL_IDLE = 0,
    OL_ALIGN,
    OL_ACCEL,
    OL_HOLD,
    OL_DECEL
};

static uint8_t ol_state = OL_IDLE;
static float   ol_wr    = 0.0f;      /* 当前速度给定 */
static float   ol_hold  = 0.0f;      /* 匀速计时 */
static float   ol_align = 0.0f;      /* 定位计时 */

#if OL_STALL_ENABLE
static float   ol_stall_t    = 0.0f; /* 堵转持续计时 */
static float   ol_prev_rad   = 0.0f; /* 上次采样的编码器累计角度 */
static uint8_t ol_stall_hit  = 0u;   /* 曾经判过堵转 (给上位机看) */
#endif

/**
***********************************************************************
* @brief:      ol_hard_fault(void)
* @retval:     uint8_t 1 = 有必须停机的硬故障
* @details:    判断是否有「必须停机」的故障。
*              刻意不含 ioff_err: 开环 V/f 不需要电流反馈, 而且 U14 没焊时
*              这个位本来就是置起的, 算进去会导致开环永远起不来。
*              也不含 enc_err: 开环用内部拖拽角度, 不用编码器。
***********************************************************************
**/
static uint8_t ol_hard_fault(void)
{
    return (uint8_t)((pm.fault.bit.un_volt != 0u) ||
                     (pm.fault.bit.ov_volt != 0u) ||
                     (pm.fault.bit.ov_curr != 0u) ||
                     (pm.fault.bit.ov_tmos != 0u) ||
                     (pm.fault.bit.ov_tcoi != 0u));
}

/* ===========================================================================
 *  硬件自检模式 (hwt_self_test_run)
 *
 *  用途: 新板焊好 / 修好之后第一次上电, **不接电机**, 先验证功率级好不好。
 *
 *  做法: 三相给三个固定且明显不同的占空比 (0.25 / 0.50 / 0.75),
 *        因为占空比固定不变, 输出端对 GND 是纯直流, 万用表直接能量。
 *
 *  期望值 (母线 11.4V 为例):
 *      输出端 1  ->  2.85 V
 *      输出端 2  ->  5.70 V
 *      输出端 3  ->  8.55 V
 *
 *  判读:
 *      三个值明显不同且都对  -> 功率级正常, 可以接电机
 *      全是 0                -> 上桥完全没工作 (VCC 没电 / 驱动坏)
 *      全等于母线电压        -> 上桥全开、驱动浮空 (也是驱动没电)
 *      有一个不对            -> 那一路坏了, 别接电机
 *
 *  ⚠️ 1. 占空比都留在 0.1~0.9 之间, 保证下桥有足够导通时间去给自举电容充电。
 *     2. 这个模式是给"不接电机"用的。接了电机会有直流电流流过绕组。
 *     3. 这里只是"每周期重写 CCR", 不走 FOC。但 ISR 里的 pmsm_state_ctrl()
 *        和 pmsm_fault_check() 照常在跑 —— 所以硬故障(欠压/过压/过流/过温)
 *        仍然会触发 pmsm_fault_stop_mode() 关掉 PWM 输出。这是对的, 别改。
 *        ⇒ 如果量出来三个都是 0, 先看 fault 位, 可能只是母线没到 8V。
 * =========================================================================== */
#define HWT_DUTY_A      0.25f
#define HWT_DUTY_B      0.50f
#define HWT_DUTY_C      0.75f

/**
***********************************************************************
* @brief:      hwt_self_test_run(void)
* @param[in]:  void
* @retval:     void
* @details:    硬件自检: 三相固定占空比输出, 用来验证功率级。主循环调用
***********************************************************************
**/
void hwt_self_test_run(void)
{
    static uint8_t started = 0u;

    /* 占空比先写进 CCR，再开输出。中断里不再改成 50%。 */
    pm.foc.dtc_a = HWT_DUTY_A;
    pm.foc.dtc_b = HWT_DUTY_B;
    pm.foc.dtc_c = HWT_DUTY_C;
    foc_pwm_run(&pm);

    pm.foc.mode   = foc_hwt_mode;
    pm.ctrl_bit   = opera;
    pm.ctrl.vq_set = 1.5f;   /* I7=1.5 表示这组固定 PWM 正在输出 */

    if (started == 0u)
    {
        foc_pwm_start();
        started = 1u;
    }
}

/**
***********************************************************************
* @brief:      openloop_test_start(void)
* @param[in]:  void
* @retval:     void
* @details:    启动开环 V/f 斜坡。有硬故障时拒绝启动
***********************************************************************
**/
void openloop_test_start(void)
{
    if (ol_hard_fault() != 0u)
    {
        return;     /* 有故障就别起了 */
    }

    ol_wr    = 0.0f;
    ol_hold  = 0.0f;
    ol_align = 0.0f;
    ol_state = OL_ALIGN;

#if OL_STALL_ENABLE
    ol_stall_t   = 0.0f;
    ol_stall_hit = 0u;
    ol_prev_rad  = mt6701.total_rad;
#endif
}

/**
***********************************************************************
* @brief:      openloop_test_stop(void)
* @param[in]:  void
* @retval:     void
* @details:    立即停止开环测试并关输出
***********************************************************************
**/
void openloop_test_stop(void)
{
    ol_state = OL_IDLE;
    ol_wr    = 0.0f;
    ol_hold  = 0.0f;
    ol_align = 0.0f;

    pm.ctrl.wr_set = 0.0f;
    pm.ctrl.vd_set = 0.0f;
    pm.ctrl.vq_set = 0.0f;
    pm.ctrl_bit   = reset;      /* 状态机收到 reset 会关 PWM (只关一次) */
}

/**
***********************************************************************
* @brief:      openloop_test_run(void)
* @param[in]:  void
* @retval:     void
* @details:    开环 V/f 测试状态机, 主循环 100Hz 调用
* @note        任何硬故障都会立刻停机; 跑完 OL_HOLD_SEC 秒自动减速停机
***********************************************************************
**/
void openloop_test_run(void)
{
    float v;

    /* 一直盯着故障: 有故障立刻停 */
    if (ol_hard_fault() != 0u)
    {
        openloop_test_stop();
        return;
    }

    if (ol_state == OL_IDLE)
    {
        return;
    }

    switch (ol_state)
    {
    case OL_ALIGN:
        ol_wr = 0.0f;
        ol_align += OL_TS;
        if (ol_align >= OL_ALIGN_SEC)
        {
            ol_state = OL_ACCEL;
        }
        break;

    case OL_ACCEL:
        ol_wr += OL_RAMP_RATE * OL_TS;
        if (ol_wr >= OL_WR_TARGET)
        {
            ol_wr    = OL_WR_TARGET;
            ol_hold  = 0.0f;
            ol_state = OL_HOLD;
        }
        break;

    case OL_HOLD:
        ol_hold += OL_TS;
        if (ol_hold >= OL_HOLD_SEC)
        {
            ol_state = OL_DECEL;
        }
        break;

    case OL_DECEL:
        ol_wr -= OL_RAMP_RATE * OL_TS;
        if (ol_wr <= 0.0f)
        {
            openloop_test_stop();
            return;
        }
        break;

    default:
        return;
    }

    /* V/f: 电压随速度线性上升 + 启动偏置 */
    v = OL_VF_RATIO * ol_wr + OL_V_MIN;

#if OL_STALL_ENABLE
    /* ---- 堵转保护: 命令转速够了但转子没跟上, 立刻停机 ----
     * 不做这个检查的话, 转子卡住时电压全加在相电阻上, 全部变成热 */
    if (ol_wr >= OL_STALL_WR_MIN)
    {
        float drad  = mt6701.total_rad - ol_prev_rad;
        float w_mea;

        if (drad < 0.0f)
        {
            drad = -drad;               /* 反转也算在动, 取模 */
        }
        w_mea = drad / OL_TS;           /* 实测机械角速度 (rad/s) */

        if (w_mea < (ol_wr * OL_STALL_RATIO))
        {
            ol_stall_t += OL_TS;
            if (ol_stall_t >= OL_STALL_SEC)
            {
                ol_stall_hit = 1u;
                openloop_test_stop();   /* 停机, 别再加热了 */
                return;
            }
        }
        else
        {
            ol_stall_t = 0.0f;          /* 转起来了, 清零 */
        }
    }
    ol_prev_rad = mt6701.total_rad;
#endif

    /* 写给定。wr_set 是机械角速度, force_volt_mode 会乘极对数得到电角速度 */
    pm.ctrl.wr_set = ol_wr;
    pm.ctrl.vd_set = 0.0f;
    pm.ctrl.vq_set = v;

    /* 必须先确认走 V/f 分支, 再切 opera。
     * pmsm_mode_ctrl() 是按 pm.foc.mode 分派的, 如果这里不是 foc_volt_mode,
     * 就会跑到电流闭环分支去 */
    pm.foc.mode = foc_volt_mode;
    pm.ctrl_bit = opera;
}

/**
***********************************************************************
* @brief:      foc_vel(pmsm_t* pm, float vel_set, float iq_set, float pos)
* @param[in]:  pm       指向 PMSM 控制结构体的指针
* @param[in]:  vel_set  速度给定 (机械角速度 rad/s)
* @param[in]:  iq_set   q 轴电流前馈 (可给 0)
* @param[in]:  pos      电角度 (rad)
* @retval:     void
* @details:    速度模式: 速度环算出 q 轴电流给定, 再交给电流环执行
***********************************************************************
**/
_RAM_FUNC void foc_vel(pmsm_t* pm, float vel_set, float iq_set, float pos)
{
    pm->foc.mode = foc_vel_mode;
    pm->ctrl.wr_set = vel_set;

    /* 外环: 速度 PI -> iq */
    foc_spd_pi_calc(pm);

    /* 前馈叠加 (可选): 上位机直接给的 iq 分量 */
    pm->ctrl.iq_set += iq_set;

    /* 内环: 电流环执行。id 给 0 (表贴式, 最大转矩/电流) */
    foc_curr(pm, CUR_ID_SET, pm->ctrl.iq_set, pos);
}

/* ===========================================================================
 *  电角度零点 e_off 自动标定 (calib_run)
 *
 *  原理: sensory_pos_calc() 里是
 *            p_e = mt6701.rad * pn + e_off
 *        p_e 就是电流环用的电角度。反过来只要把电压矢量钉在一个已知的
 *        电角度 CALIB_ANG 上, 转子会被吸到那个方向并停住, 这时读编码器
 *        就能解出 e_off:
 *            e_off = CALIB_ANG - (机械角 * pn)
 *
 *  为什么用 total_rad 而不是 rad:
 *        代码里 p_e 用的是 rad (0~2pi, 会跳变), 直接对 rad 取平均在过零点
 *        会算出垃圾。而 pn 是整数, (total_rad - rad) 一定是 2*pi 的整数倍,
 *        乘 pn 之后模 2*pi 完全一样 —— 所以用 total_rad 算, 结果等价且不跳变。
 *
 *  为什么要锁两个相差 180 度电角度的位置:
 *        两次算出来的 e_off 必须一致。差得多就说明转子没稳定、有负载、
 *        或者编码器/相序有问题 —— 这时候标定结果不能用。
 *
 *  ⚠️ 标定必须空载。轴上有负载会把转子拖偏, 标出来的角是错的。
 * =========================================================================== */
#define CALIB_TS        0.01f               /* 主循环周期 (秒) */
#define CALIB_V         2.0f                /* 标定电压 (V): 2V/3.1Ω=0.65A, 短时安全 */
#define CALIB_SETTLE_S  2.0f                /* 每个角度等转子稳定的时间 */
#define CALIB_AVG_S     0.8f                /* 取平均的时间 */
#define CALIB_ANG_A     0.0f                /* 第一次锁的电角度 */
#define CALIB_ANG_B     3.14159265358979f   /* 第二次, 相差 180 度电角度 */

enum
{
    CALIB_IDLE = 0,
    CALIB_A_SETTLE,
    CALIB_A_MEAS,
    CALIB_B_SETTLE,
    CALIB_B_MEAS,
    CALIB_DONE
};

/* 全部用 volatile 且非 static, 方便 gdb/openocd 直接读结果 */
volatile uint8_t calib_step    = CALIB_IDLE;
volatile float   calib_rad_a   = 0.0f;  /* 锁 A 角时测到的机械角 (rad) */
volatile float   calib_rad_b   = 0.0f;  /* 锁 B 角时测到的机械角 (rad) */
volatile float   calib_e_off_a = 0.0f;  /* 由 A 算出的 e_off */
volatile float   calib_e_off_b = 0.0f;  /* 由 B 算出的 e_off */
volatile float   calib_e_off   = 0.0f;  /* 最终采用的 e_off */

static float calib_t       = 0.0f;
static float calib_rad_beg = 0.0f;

/**
***********************************************************************
* @brief:      calib_wrap(float x)
* @param[in]:  x  任意弧度
* @retval:     归一化到 [0, 2pi) 的角度
* @details:    电角度零点标定用，避免累加后跑出 0~2pi
***********************************************************************
**/
static float calib_wrap(float x)
{
    while (x < 0.0f)   { x += M_2PI; }
    while (x >= M_2PI) { x -= M_2PI; }
    return x;
}

/**
***********************************************************************
* @brief:      calib_run(void)
* @param[in]:  void
* @retval:     void
* @details:    电角度零点自动标定, 主循环 100Hz 调用。标定完自动写回 pm.para.e_off
***********************************************************************
**/
void calib_run(void)
{
    /* 母线没起来就别动 —— 上电后 12V 要一点时间才稳 */
    if (pm.foc.vbus < 8.0f)
    {
        return;
    }

    /* 必须先走 V/f 分支, 再把 ctrl_bit 打到 opera, 状态机会自动开 PWM */
    pm.foc.mode    = foc_volt_mode;
    pm.ctrl.vd_set = 0.0f;
    pm.ctrl.wr_set = 0.0f;      /* wr=0 -> 中断里 pos_acc=0, drag_pe 不会被累加 */
    pm.ctrl_bit    = opera;

    switch (calib_step)
    {
    case CALIB_IDLE:                        /* 第一次进来, 开跑 */
        calib_t    = 0.0f;
        calib_step = CALIB_A_SETTLE;
        break;

    case CALIB_A_SETTLE:                    /* 锁 A 角, 等转子被吸住 */
        pm.ctrl.vq_set  = CALIB_V;
        pm.ctrl.drag_pe = CALIB_ANG_A;
        calib_t += CALIB_TS;
        if (calib_t >= CALIB_SETTLE_S)
        {
            calib_t       = 0.0f;
            calib_rad_beg = mt6701.total_rad;
            calib_step    = CALIB_A_MEAS;
        }
        break;

    case CALIB_A_MEAS:                      /* 转子已停, 取平均 */
        pm.ctrl.vq_set  = CALIB_V;
        pm.ctrl.drag_pe = CALIB_ANG_A;
        calib_t += CALIB_TS;
        if (calib_t >= CALIB_AVG_S)
        {
            calib_rad_a   = 0.5f * (calib_rad_beg + mt6701.total_rad);
            calib_e_off_a = calib_wrap(CALIB_ANG_A - calib_rad_a * pm.para.pn);
            calib_t       = 0.0f;
            calib_step    = CALIB_B_SETTLE;
        }
        break;

    case CALIB_B_SETTLE:                    /* 换到 B 角 (相差 180 度电角度) */
        pm.ctrl.vq_set  = CALIB_V;
        pm.ctrl.drag_pe = CALIB_ANG_B;
        calib_t += CALIB_TS;
        if (calib_t >= CALIB_SETTLE_S)
        {
            calib_t       = 0.0f;
            calib_rad_beg = mt6701.total_rad;
            calib_step    = CALIB_B_MEAS;
        }
        break;

    case CALIB_B_MEAS:
        pm.ctrl.vq_set  = CALIB_V;
        pm.ctrl.drag_pe = CALIB_ANG_B;
        calib_t += CALIB_TS;
        if (calib_t >= CALIB_AVG_S)
        {
            calib_rad_b   = 0.5f * (calib_rad_beg + mt6701.total_rad);
            calib_e_off_b = calib_wrap(CALIB_ANG_B - calib_rad_b * pm.para.pn);
            calib_e_off   = calib_e_off_a;          /* 采用 A 的结果 */
            pm.para.e_off = calib_e_off;            /* 直接生效, 不用手抄 */
            calib_step    = CALIB_DONE;
        }
        break;

    case CALIB_DONE:
    default:
        pm.ctrl_bit = reset;    /* 标定完立刻关输出, 别再加热电机 */
        break;
    }
}

/* ===========================================================================
 *  电流环静态测试 (curr_test_run)
 *
 *  用途: 第一次跑电流闭环时, 先不转, 只验证「环闭合、符号对、采样相位对」。
 *
 *  做法: 把电角度**钉死**在 CT_ANGLE, 命令 id=0 / iq=CT_IQ_SET。
 *        因为角度不跟着编码器走, 电流矢量在空间里是固定的:
 *
 *            定子磁动势固定在 CT_ANGLE + 90°
 *                    ↓
 *            转子 d 轴被吸过去, 停在这个方向 (转一下就不动了)
 *                    ↓
 *            此时电流矢量正好落在转子的 d 轴上 -> 不产生转矩 -> 稳定
 *
 *        稳定后 PI 应该把 i_d 调到 0、i_q 调到 CT_IQ_SET。
 *
 *  怎么看结果:
 *        ct_i_d  应该 ≈ 0
 *        ct_i_q  应该 ≈ CT_IQ_SET
 *        ct_v_d / ct_v_q 是 PI 输出, 不应顶到限幅
 *
 *  ⚠️ 如果 PI 极性接反 / 采样相位错, 电流会自己发散到电压限幅。
 *     所以这里加了 CT_ABORT_A 兜底: 任一相电流超过就立刻停机。
 * =========================================================================== */
#define CT_TS        0.01f      /* 主循环周期 (秒) */
#define CT_IQ_SET    0.15f      /* q 轴电流给定 (A)。先给小值, 验证通了再加大 */
#define CT_ANGLE     0.0f       /* 钉死的电角度 (rad) */
#define CT_HOLD_S    300.0f     /* 跑这么久后自动停机 (诊断期间放长, 方便看波形) */
#define CT_ABORT_A   0.60f      /* 电流超过这个值就立刻停机 (兜底保护) */

volatile uint8_t ct_done = 0u;  /* 1 = 测试结束 */
volatile float   ct_i_d  = 0.0f;
volatile float   ct_i_q  = 0.0f;
volatile float   ct_v_d  = 0.0f;
volatile float   ct_v_q  = 0.0f;
volatile float   ct_ia   = 0.0f;    /* 三相实测电流, 用来判断采样是否正常 */
volatile float   ct_ib   = 0.0f;
volatile float   ct_ic   = 0.0f;
volatile float   ct_t    = 0.0f;

/**
***********************************************************************
* @brief:      curr_test_run(void)
* @param[in]:  void
* @retval:     void
* @details:    电流环静态测试, 固定电角度 + 电流闭环。主循环 100Hz 调用
***********************************************************************
**/
void curr_test_run(void)
{
    /* 已经结束了就别再输出 */
    if (ct_done != 0u)
    {
        pm.ctrl_bit = reset;
        return;
    }

    /* 母线没起来就别动 */
    if (pm.foc.vbus < 8.0f)
    {
        return;
    }

    /* 兜底: 电流异常立刻停机, 防止 PI 极性错误时发散 */
    if ((pm.foc.i_a > CT_ABORT_A) || (pm.foc.i_a < -CT_ABORT_A) ||
        (pm.foc.i_b > CT_ABORT_A) || (pm.foc.i_b < -CT_ABORT_A) ||
        (pm.foc.i_c > CT_ABORT_A) || (pm.foc.i_c < -CT_ABORT_A))
    {
        ct_done     = 1u;
        pm.ctrl_bit = reset;
        return;
    }

    /* 到时间就停 */
    ct_t += CT_TS;
    if (ct_t >= CT_HOLD_S)
    {
        ct_done     = 1u;
        pm.ctrl_bit = reset;
        return;
    }

    pm.ctrl_bit = opera;

    /* 电角度钉死, 不走编码器 —— 这样空间矢量固定, 转子停住不转 */
    foc_curr(&pm, 0.0f, CT_IQ_SET, CT_ANGLE);

    /* 把中间量留下来给 SWD 读 */
    ct_i_d = pm.foc.i_d;
    ct_i_q = pm.foc.i_q;
    ct_v_d = pm.foc.v_d;
    ct_v_q = pm.foc.v_q;
    ct_ia  = pm.foc.i_a;
    ct_ib  = pm.foc.i_b;
    ct_ic  = pm.foc.i_c;
}

/* ===========================================================================
 *  真 FOC 测试: 编码器角度 + 电流环 (curr_enc_test_run)
 *
 *  和静态测试的唯一区别: 电角度不再是钉死的, 而是来自编码器
 *            p_e = mt6701.rad * pn + e_off
 *  这就是真正的磁场定向控制。
 *
 *  为什么这次不会像静态测试那样振:
 *        电流矢量跟着转子走, 始终落在转子的 dq 轴上, 是纯转矩
 *        没有"磁弹簧", 也就没有那个 13Hz 的谐振
 *
 *  ⚠️ 安全问题: 给 iq 就是给转矩。这台电机 J≈1e-5, 0.15A 的角加速度是
 *     Kt*iq/J = 0.0651*0.15/1e-5 = 977 rad/s^2 (9300 rpm/s) —— 会飞车。
 *     所以这里加了简单的转速上限: 超过 CE_WMAX 就把 iq 撤成 0 让它滑行。
 *     这是个 bang-bang 限速, 不是速度环, 目的是安全地验证角度链路。
 *
 *  怎么看结果:
 *        ce_wr   实测机械角速度。给正 iq 应该往正方向涨
 *        ce_i_q  应该跟着 iq 给定走
 *        ce_done 1 = 测试结束
 * =========================================================================== */
#define CE_IQ        0.20f      /* q 轴电流给定 (A)。0.03 推不动齿槽, 提到 0.20 */
#define CE_WMAX      40.0f      /* 转速上限 (rad/s) = 382 rpm, 超了撤 iq */
#define CE_HOLD_S    25.0f      /* 跑这么久后停机 */
#define CE_ABORT_A   0.60f      /* 任一相电流超过就停机 */

volatile uint8_t ce_done = 0u;
volatile float   ce_wr   = 0.0f;    /* 实测机械角速度 (rad/s) */
volatile float   ce_i_q  = 0.0f;
volatile float   ce_i_d  = 0.0f;
volatile float   ce_p_e  = 0.0f;    /* 编码器算出来的电角度 */
volatile float   ce_rad  = 0.0f;    /* 机械角 */
volatile float   ce_t    = 0.0f;

static float ce_rad_prev = 0.0f;
static float ce_iq_now   = 0.0f;

/**
***********************************************************************
* @brief:      curr_enc_test_run(void)
* @param[in]:  void
* @retval:     void
* @details:    真 FOC: 编码器电角度 + 电流环 + 转速上限。主循环 100Hz 调用
***********************************************************************
**/
void curr_enc_test_run(void)
{
    float drad;

    if (ce_done != 0u)
    {
        pm.ctrl_bit = reset;
        return;
    }

    if (pm.foc.vbus < 8.0f)
    {
        return;
    }

    /* 兜底: 电流异常立刻停机 */
    if ((pm.foc.i_a > CE_ABORT_A) || (pm.foc.i_a < -CE_ABORT_A) ||
        (pm.foc.i_b > CE_ABORT_A) || (pm.foc.i_b < -CE_ABORT_A) ||
        (pm.foc.i_c > CE_ABORT_A) || (pm.foc.i_c < -CE_ABORT_A))
    {
        ce_done     = 1u;
        pm.ctrl_bit = reset;
        return;
    }

    /* 到时间就停 */
    ce_t += CT_TS;
    if (ce_t >= CE_HOLD_S)
    {
        ce_done     = 1u;
        pm.ctrl_bit = reset;
        return;
    }

    pm.ctrl_bit = opera;

    /* ---- 1. 编码器 -> 电角度 ---- */
    sensory_pos_calc(&pm);              /* p_e = mt6701.rad * pn + e_off */

    /* ---- 2. 实测转速 (用 total_rad 差分, 不依赖 foc 内部计算) ---- */
    drad        = mt6701.total_rad - ce_rad_prev;
    ce_rad_prev = mt6701.total_rad;
    ce_wr       = drad / CT_TS;

    /* ---- 3. 简单限速: 超了就撤 iq 让它滑行 ---- */
    if ((ce_wr > CE_WMAX) || (ce_wr < -CE_WMAX))
    {
        ce_iq_now = 0.0f;
    }
    else
    {
        ce_iq_now = CE_IQ;
    }

    /* ---- 4. 电流环, 用真实电角度 ---- */
    foc_curr(&pm, 0.0f, ce_iq_now, pm.foc.p_e);

    /* ---- 5. 记录 ---- */
    ce_i_q = pm.foc.i_q;
    ce_i_d = pm.foc.i_d;
    ce_p_e = pm.foc.p_e;
    ce_rad = pm.foc.p_e;                /* 预留 */
}
