/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 控制器库 —— 实现 common.h 中声明的 PID 接口
  *
  *  设计说明:
  *    1. 并联式 PI(D):  out = kp*e + ki*Σe + kd*Δe
  *       - ki 是「每个采样周期累加 ki*e」的离散增益, 不是连续域的 Ki
  *       - 电流环 ts = 50us (20kHz), 速度环 ts = 100us (10kHz)
  *
  *    2. 抗积分饱和 (anti-windup) 是这里的重点:
  *       - parallel_pid_ctrl : 积分项单独限幅, 简单可靠, 够用
  *       - serial_pid_ctrl   : 条件积分 —— 输出饱和且误差还在往同方向推时,
  *                             撤销本次积分。比单纯限幅更不容易「退饱和慢」
  *       只限输出不限积分是错的, 会导致退饱和时严重超调。
  *
  *    3. 微分项 kd=0 时完全跳过计算, 20kHz 下能省一点时间
  ******************************************************************************
  */
#include "common.h"

/**
***********************************************************************
* @brief:      sat1_datf(float val, float up, float low)
* @param[in]:  val  待限幅的值
* @param[in]:  up   上限
* @param[in]:  low  下限
* @retval:     float 限幅后的值
* @details:    单点饱和限幅, 把 val 夹在 [low, up] 区间内
***********************************************************************
**/
float sat1_datf(float val, float up, float low)
{
    if (val > up)
    {
        return up;
    }
    if (val < low)
    {
        return low;
    }
    return val;
}

/**
***********************************************************************
* @brief:      pid_para_init(pid_para_t* pid)
* @param[in]:  pid  PID 结构体指针
* @retval:     void
* @details:    PID 结构体清零, 所有增益与状态复位
***********************************************************************
**/
void pid_para_init(pid_para_t* pid)
{
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;

    pid->kfp = 0.0f;
    pid->kf_damp = 0.0f;

    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;

    pid->i_term_max = 0.0f;
    pid->i_term_min = 0.0f;

    pid->ts = 0.0f;

    pid->ref_value = 0.0f;
    pid->fback_value = 0.0f;

    pid->error = 0.0f;
    pid->pre_err = 0.0f;

    pid->out_min = 0.0f;
    pid->out_max = 0.0f;

    pid->out_value = 0.0f;
}

/**
***********************************************************************
* @brief:      pid_limit_init(pid_para_t* pid, ...)
* @param[in]:  pid      PID 结构体指针
* @param[in]:  i_max    积分项上限
* @param[in]:  i_min    积分项下限
* @param[in]:  out_max  输出上限
* @param[in]:  out_min  输出下限
* @retval:     void
* @details:    设置积分项限幅与输出限幅。积分限幅用来抗饱和, 输出限幅是物理约束
***********************************************************************
**/
void pid_limit_init(pid_para_t* pid, float i_max, float i_min, float out_max, float out_min)
{
    pid->i_term_max = i_max;
    pid->i_term_min = i_min;
    pid->out_max = out_max;
    pid->out_min = out_min;
}

/**
***********************************************************************
* @brief:      pid_clear(pid_para_t* pid)
* @param[in]:  pid  PID 结构体指针
* @retval:     void
* @details:    清 PID 内部状态 (积分项/微分项/误差), 保留增益与限幅设置。
*              模式切换、故障恢复后要调一次, 否则会带着旧积分冲出去
***********************************************************************
**/
void pid_clear(pid_para_t* pid)
{
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;

    pid->error = 0.0f;
    pid->pre_err = 0.0f;

    pid->out_value = 0.0f;
}

/**
***********************************************************************
* @brief:      pid_reset(pid_para_t* pid, float kp, float ki, float kd)
* @param[in]:  pid  PID 结构体指针
* @param[in]:  kp   比例增益
* @param[in]:  ki   积分增益
* @param[in]:  kd   微分增益
* @retval:     void
* @details:    设置增益并清状态
***********************************************************************
**/
void pid_reset(pid_para_t* pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid_clear(pid);
}

/**
***********************************************************************
* @brief:      parallel_pid_ctrl(pid_para_t* pid, float ref, float fback)
* @param[in]:  pid    PID 结构体指针
* @param[in]:  ref    参考值
* @param[in]:  fback  反馈值
* @retval:     float  控制器输出 (已限幅)
* @details:    并联式 PI(D)。积分项每周期累加后单独限幅, 实现抗积分饱和
***********************************************************************
**/
float parallel_pid_ctrl(pid_para_t* pid, float ref, float fback)
{
    pid->ref_value = ref;
    pid->fback_value = fback;
    pid->error = ref - fback;

    /* 比例项 */
    pid->p_term = pid->kp * pid->error;

    /* 积分项: 先累加再限幅 -> 积分饱和被夹住 */
    pid->i_term += pid->ki * pid->error;
    pid->i_term = sat1_datf(pid->i_term, pid->i_term_max, pid->i_term_min);

    /* 微分项: kd=0 时跳过 */
    if (pid->kd != 0.0f)
    {
        pid->d_term = pid->kd * (pid->error - pid->pre_err);
    }
    else
    {
        pid->d_term = 0.0f;
    }
    pid->pre_err = pid->error;

    pid->out_value = sat1_datf(pid->p_term + pid->i_term + pid->d_term,
                               pid->out_max, pid->out_min);

    return pid->out_value;
}

/**
***********************************************************************
* @brief:      serial_pid_ctrl(pid_para_t* pid, float ref, float fback)
* @param[in]:  pid    PID 结构体指针
* @param[in]:  ref    参考值
* @param[in]:  fback  反馈值
* @retval:     float  控制器输出 (已限幅)
* @details:    串级式 PI(D) + 条件积分抗饱和。
*              当输出已饱和、且误差还在把输出往同一个方向推时, 撤销本次积分,
*              避免积分越攒越多导致的退饱和超调
***********************************************************************
**/
float serial_pid_ctrl(pid_para_t* pid, float ref, float fback)
{
    float out;

    pid->ref_value = ref;
    pid->fback_value = fback;
    pid->error = ref - fback;

    /* 积分项限幅, 再累加 */
    pid->i_term = sat1_datf(pid->i_term, pid->i_term_max, pid->i_term_min);
    pid->i_term += pid->ki * pid->error;

    /* 比例项 */
    pid->p_term = pid->kp * pid->error;

    /* 微分项: kd=0 时跳过 */
    if (pid->kd != 0.0f)
    {
        pid->d_term = pid->kd * (pid->error - pid->pre_err);
    }
    else
    {
        pid->d_term = 0.0f;
    }
    pid->pre_err = pid->error;

    out = pid->p_term + pid->i_term + pid->d_term;

    /* 条件积分: 输出饱和且误差同向 -> 撤销本次积分 */
    if (((out > pid->out_max) && (pid->error > 0.0f)) ||
        ((out < pid->out_min) && (pid->error < 0.0f)))
    {
        pid->i_term -= pid->ki * pid->error;
        out = pid->p_term + pid->i_term + pid->d_term;
    }

    pid->out_value = sat1_datf(out, pid->out_max, pid->out_min);

    return pid->out_value;
}

/**
***********************************************************************
* @brief:      serial_pid_ctrl1(pid_para_t* pid, float ref, float fback, float i_max, float out_max)
* @param[in]:  pid      PID 结构体指针
* @param[in]:  ref      参考值
* @param[in]:  fback    反馈值
* @param[in]:  i_max    积分项限幅 (对称)
* @param[in]:  out_max  输出限幅 (对称)
* @retval:     float    控制器输出
* @details:    serial_pid_ctrl 的便捷版本: 用对称限幅 + 每周期刷新限幅值。
*              适合限幅值会随工况变化 (例如随母线电压变化) 的场合
***********************************************************************
**/
float serial_pid_ctrl1(pid_para_t* pid, float ref, float fback, float i_max, float out_max)
{
    pid->i_term_max = i_max;
    pid->i_term_min = -i_max;

    pid->out_max = out_max;
    pid->out_min = -out_max;

    return serial_pid_ctrl(pid, ref, fback);
}

/**
***********************************************************************
* @brief:      pdff_ctrl(pid_para_t* pid, float ref, float fback)
* @param[in]:  pid    PID 结构体指针
* @param[in]:  ref    参考值
* @param[in]:  fback  反馈值
* @retval:     float  控制器输出 (已限幅)
* @details:    伪微分反馈前馈。微分作用在反馈上，参考值阶跃时不会产生微分冲击。
*              适合速度环这种参考会突变的场合。
*              注意: 这里 pre_err 存的是上一次的反馈值, 不是误差
***********************************************************************
**/
float pdff_ctrl(pid_para_t* pid, float ref, float fback)
{
    pid->ref_value = ref;
    pid->fback_value = fback;
    pid->error = ref - fback;

    /* 比例项 */
    pid->p_term = pid->kp * pid->error;

    /* 积分项: 累加后限幅 */
    pid->i_term += pid->ki * pid->error;
    pid->i_term = sat1_datf(pid->i_term, pid->i_term_max, pid->i_term_min);

    /* 微分项: 对反馈求差分, 而不是对误差 */
    if (pid->kd != 0.0f)
    {
        pid->d_term = -pid->kd * (fback - pid->pre_err);
    }
    else
    {
        pid->d_term = 0.0f;
    }
    pid->pre_err = fback;

    pid->out_value = sat1_datf(pid->p_term + pid->i_term + pid->d_term,
                               pid->out_max, pid->out_min);

    return pid->out_value;
}
