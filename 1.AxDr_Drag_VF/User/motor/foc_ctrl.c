#include "common.h"
#include "modlue.h"
/**
***********************************************************************
* @brief:      pwmv2_cmp1_callback(void)
* @param[in]:  void
* @retval:     void
* @details:    PWM主中断回调函数，读取ADC值，计算电流，执行FOC参数计算、故障检测和状态控制
***********************************************************************
**/
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    foc_adc_sample(&pm);
    foc_para_calc(&pm);
    pmsm_fault_check(&pm);   /* 先查故障: 有故障会把 ctrl_bit 打到 reset 停机 */
    pmsm_state_ctrl(&pm);
	// vofa_start();   /* 暂时关闭: ISR 每 50us 发一帧会和主循环的编码器调试帧互相干扰 */
}

/**
***********************************************************************
* @brief:      pmsm_state_ctrl(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    PMSM 状态机控制，根据当前状态执行启动、预充、复位和运行等操作
***********************************************************************
**/
_RAM_FUNC void pmsm_state_ctrl(pmsm_t* pm)
{
    /* 保证「关输出」只执行一次。
     * 原来 reset 分支每 50us 都会调一次 HAL_TIM_PWM_Stop —— 只要 ctrl_bit
     * 被置成 reset (故障停机 / 上位机复位), 就会在 20kHz 中断里空耗 CPU */
    static uint8_t pwm_stopped = 0u;

    // State machine for PMSM control
    switch (pm->ctrl_bit)
    {
    case start:
        // Initialize PWM and transition to precharge state
        pwm_stopped = 0u;
        foc_pwm_start();
        foc_pwm_duty_set(pm);
        break;
    case reset:
         //Reset all controllers and stop PWM
        if (pwm_stopped == 0u)
        {
            foc_pwm_stop();
            pwm_stopped = 1u;
        }
        break;
    case opera:
        // Normal operation mode control
        pmsm_mode_ctrl(pm);
        break;
    default:
        break;
    }
}

/**
***********************************************************************
* @brief:      pmsm_mode_ctrl(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    PMSM模式控制，根据当前系统模式选择不同的控制策略
***********************************************************************
**/
_RAM_FUNC void pmsm_mode_ctrl(pmsm_t* pm)
{
	/* 默认 pm->foc.mode == foc_volt_mode (枚举值 0), 保持原来的 V/f 开环行为。
	 * 只有上位机显式把 foc.mode 切过去, 才会走闭环分支 */
	if (pm->foc.mode == foc_volt_mode)
	{
		force_volt_mode(pm); // V/f control mode
	}
	else if (pm->foc.mode == foc_vel_mode)
	{
		/* 速度闭环: 先取电角度, 无故障才跑 */
		sensory_pos_calc(pm);
		if (pm->fault.all == 0u)
		{
			foc_vel(pm, pm->ctrl.wr_set, 0.0f, pm->foc.p_e);
		}
	}
	else
	{
		/* 电流闭环 (foc_curr_mode / foc_pos_mode) */
		force_curr_mode(pm);
	}
}

/**
***********************************************************************
* @brief:      force_volt_mode(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    V/f 控制模式下的电压控制，计算电角速度和位置，执行电压控制
***********************************************************************
**/
_RAM_FUNC void force_volt_mode(pmsm_t* pm)
{
    // Calculate electrical angular velocity from reference speed
    pm->ctrl.we_set = pm->ctrl.wr_set * pm->para.pn;
    // Calculate position increment per FOC period
    pm->ctrl.pos_acc = pm->ctrl.we_set * pm->period.foc_ts;
    // Update electrical angle
    pm->ctrl.drag_pe += pm->ctrl.pos_acc;
    // Wrap angle to [0, 2π)
    wrap_0_2pi(pm->ctrl.drag_pe);
    // Apply voltage control
    foc_volt(pm, pm->ctrl.vd_set, pm->ctrl.vq_set, pm->ctrl.drag_pe);
}


