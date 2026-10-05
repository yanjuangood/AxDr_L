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
