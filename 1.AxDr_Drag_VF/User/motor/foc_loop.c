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
    pm->foc.p_e = mt6701.rad * pm->para.pn + pm->para.e_off;

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
  *  ⚠️ 安全须知 (这块板 + 这类电机的组合很敏感):
  *     - 本板没有任何硬件过流保护 (FD6288Q 的 ITRIP/FAULT 是 NC)
  *     - 2312 电机 Rs 只有 0.203 欧。开环是恒压驱动, 低速时电流完全由
  *       「电压 / 相电阻」决定, 没有任何反馈去限制它:
  *           0.15V / 0.203 = 0.74 A
  *           0.50V / 0.203 = 2.46 A
  *           1.50V / 0.203 = 7.39 A
  *     - 如果电源不能限流, 强烈建议串一个 2 欧 / 10W 的功率电阻在母线上,
  *       或者用带限流的可调电源。开机瞬间盯住电源电流表。
  *
  *  V/f 曲线怎么定的:
  *     相电压 = 反电动势 + 电阻压降
  *            = wr * pn * flux + I * Rs
  *     本电机 pn*flux = 7 * 0.0065 = 0.0455 V/(rad/s)
  *     取斜率 0.050 (略高于反电动势常数, 留一点转矩余量), 再加一个小偏置
  *     克服静摩擦。这样在任意转速下电流都稳定在 0.5~1.5A, 不会失控。
  ******************************************************************************
  */

/* V/f 斜率 (V per rad/s)。
 * 2804: pn*flux = 7*0.0062 = 0.0434 V/(rad/s) 是反电动势常数。
 * 取 0.048, 略高于它, 保证速度上去后还有电流余量。
 *
 * ⚠️ 这台电机电阻大 (2.55Ω), 电压大部分被电阻吃掉, 所以 vq 里
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

/* 匀速保持时间 (秒), 之后自动减速停机。
 * 开环跑 6 秒再自动停, 避免忘记断电导致电机长时间堵转发烫 */
#define OL_HOLD_SEC     6.0f

/* 主循环调用周期 (秒)。main.c 里是 HAL_Delay(10) */
#define OL_TS           0.01f

/* 状态机 */
enum
{
    OL_IDLE = 0,
    OL_ACCEL,
    OL_HOLD,
    OL_DECEL
};

static uint8_t ol_state = OL_IDLE;
static float   ol_wr    = 0.0f;      /* 当前速度给定 */
static float   ol_hold  = 0.0f;      /* 匀速计时 */

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
    ol_state = OL_ACCEL;
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
