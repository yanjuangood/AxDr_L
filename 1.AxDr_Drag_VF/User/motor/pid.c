/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 控制器库 —— 实现 common.h 中声明的 PID 接口
  *
  *  设计说明:
 *  和桌面工程 1.AxDr_Drag_VF 里编出来的 pid.o 同一套:
 *    serial_pid_ctrl :  i += ki * (kp*e) * ts,  输出 kp*e + i
 *    parallel_pid_ctrl: i += ki * e * ts, 积分上限是输出还剩的空间
 *    pdff_ctrl        : 比例用 kfp*参考 - kf_damp*反馈, 积分和 parallel 一样
 *  ki 是连续域增益, 函数内部乘 ts。限幅用 i_term_min/max 和 out_min/max。
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
* @details:    并联 PI。积分按 ki*误差*ts 累加。
*              积分上下限每拍改成「输出限幅减去比例项」, 负数一侧夹到 0,
*              这样比例已经顶满时积分不再往同方向加。
***********************************************************************
**/
_RAM_FUNC float parallel_pid_ctrl(pid_para_t* pid, float ref, float fback)
{
    float room;

    pid->ref_value = ref;
    pid->fback_value = fback;
    pid->error = ref - fback;

    pid->p_term = pid->kp * pid->error;
    pid->d_term = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;

    room = pid->out_max - pid->p_term;
    pid->i_term_max = (room > 0.0f) ? room : 0.0f;
    room = pid->out_min - pid->p_term;
    pid->i_term_min = (room < 0.0f) ? room : 0.0f;

    pid->i_term += pid->ki * pid->error * pid->ts;
    pid->i_term = sat1_datf(pid->i_term, pid->i_term_max, pid->i_term_min);

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;
    pid->out_value = sat1_datf(pid->out_value, pid->out_max, pid->out_min);
    return pid->out_value;
}

/**
***********************************************************************
* @brief:      serial_pid_ctrl(pid_para_t* pid, float ref, float fback)
* @param[in]:  pid    PID 结构体指针
* @param[in]:  ref    参考值
* @param[in]:  fback  反馈值
* @retval:     float  控制器输出 (已限幅)
* @details:    串联 PI, 和桌面工程 pid.o 里的 serial_pid_ctrl 一致。
*              p = kp * e
*              i += ki * p * ts
*              输出 = p + i, 积分和输出各自限幅。没有微分。
***********************************************************************
**/
_RAM_FUNC float serial_pid_ctrl(pid_para_t* pid, float ref, float fback)
{
    pid->ref_value = ref;
    pid->fback_value = fback;
    pid->error = ref - fback;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->p_term * pid->ts;
    pid->i_term = sat1_datf(pid->i_term, pid->i_term_max, pid->i_term_min);

    pid->out_value = pid->p_term + pid->i_term;
    pid->out_value = sat1_datf(pid->out_value, pid->out_max, pid->out_min);
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
* @details:    比例作用在 kfp*参考 - kf_damp*反馈 上, 积分仍按误差累加。
*              积分限幅方式和 parallel_pid_ctrl 相同。
***********************************************************************
**/
_RAM_FUNC float pdff_ctrl(pid_para_t* pid, float ref, float fback)
{
    float room;

    pid->ref_value = ref;
    pid->fback_value = fback;
    pid->error = ref - fback;
    pid->p_term = pid->kp * (pid->kfp * ref - pid->kf_damp * fback);

    room = pid->out_max - pid->p_term;
    pid->i_term_max = (room > 0.0f) ? room : 0.0f;
    room = pid->out_min - pid->p_term;
    pid->i_term_min = (room < 0.0f) ? room : 0.0f;

    pid->i_term += pid->ki * pid->error * pid->ts;
    pid->i_term = sat1_datf(pid->i_term, pid->i_term_max, pid->i_term_min);

    pid->d_term = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;
    pid->out_value = sat1_datf(pid->out_value, pid->out_max, pid->out_min);
    return pid->out_value;
}
