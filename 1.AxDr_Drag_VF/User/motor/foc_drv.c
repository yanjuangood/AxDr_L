#include "common.h"
#include "stm32g4xx_ll_tim.h"

_RAM_DATA pmsm_t pm;


/**
***********************************************************************
* @brief:      pmsm_board_init(void)
* @param[in]:  void
* @retval:     void
* @details:    驱动板参数初始化，包括电压、电流、分压电阻，放大倍数等相关参数的设置
***********************************************************************
**/
void pmsm_board_init(void)
{
    pm.board.v_ref = 3.3f;
    pm.board.v_adc = 4096.0f;

    pm.board.i_res = 0.001f;
    pm.board.i_op = 20.0f;

    pm.board.v1_res = 20000.0f; //
    pm.board.v2_res = 1000.0f;

    pm.board.v_op = (pm.board.v1_res + pm.board.v2_res) / pm.board.v2_res;
    pm.board.i_ratio = pm.board.v_ref / pm.board.v_adc / pm.board.i_res / pm.board.i_op;
    pm.board.v_ratio = pm.board.v_ref / pm.board.v_adc * pm.board.v_op;

    pm.board.i_max = pm.board.v_adc * pm.board.i_ratio * 0.5f;
    pm.board.v_max = pm.board.v_adc * pm.board.v_ratio;

    pm.board.Rt_Mos = 10000.0f;
    pm.board.Rt_Mos_res = 10000.0f;
    pm.board.Rt_Mos_Ka = 273.15f;
    pm.board.Rt_Mos_B = 3500.0f;    /* 硬件 NTC: HNTC0603-103F3450FA */

    pm.board.Rt_rotor = 10000.0f;
    pm.board.Rt_rotor_res = 10000.0f;
    pm.board.Rt_rotor_Ka = 273.15f;
    pm.board.Rt_rotor_B = 3500.0f;  /* 硬件 NTC: HNTC0603-103F3450FA */

    /* 1us。原先写了 0.5us 但从未写进 TIM1->BDTR，死区实际是 0。
     * 上电后 start 状态就以 50% 互补 PWM 开关，每个沿上下管直通，空载 MOS 也会烫。 */
    pm.board.dead_time = 1.0f;
}

/**
 * 把 pm.board.dead_time (单位 us) 写进 TIM1 死区寄存器。
 * 必须在三相 PWM 使能之前调用。定时器时钟在 APB2 分频为 1 时等于 SystemCoreClock。
 */
static void foc_deadtime_apply(void)
{
    uint32_t dt_ns = (uint32_t)(pm.board.dead_time * 1000.0f);
    uint8_t dtg = (uint8_t)__LL_TIM_CALC_DEADTIME(SystemCoreClock,
                                                  LL_TIM_CLOCKDIVISION_DIV1,
                                                  dt_ns);
    HAL_TIMEx_ConfigDeadTime(&htim1, dtg);
}

/**
***********************************************************************
* @brief:      pmsm_2804_init(void)
* @param[in]:  void
* @retval:     void
* @details:    2804 云台电机参数初始化
*
*  数据来源: 电机规格表
*      电机型号  2804          极对数   7 (12N14P)
*      线电阻    5.1 Ω         线电感   2.8 mH
*      磁链      0.0035 Wb     转速常数 220 KV
*      额定电流  0.5 A         堵转电流 1.8 A
*      扭矩      0.03 Nm       最大转速 2700 RPM @ 12V
*
*  ⚠️ 三个换算必须做对, 否则 FOC 全错:
*    1. 手册的"线电阻/线电感"是线间值, FOC 用的是相值 -> 除以 2
*           Rs = 5.1/2 = 2.55 Ω      Ls = 2.8mH/2 = 1.4 mH
*    2. 手册的"磁链 0.0035Wb"是线值, FOC 要相值 (λ 满足 E_phase = ωe*λ)
*       用 KV 和最大转速、扭矩三路交叉验证都指向 λ ≈ 0.006:
*           用 KV     : Ke = 9.549/220 = 0.0434 -> λ = 0.0434/7 = 0.0062
*           用最大转速: 12V/2700rpm = 12/282.7 = 0.0425 -> λ = 0.0061
*           用扭矩    : Kt = 0.03/0.5 = 0.06 -> λ = 0.06/(1.5*7) = 0.0057
*           0.0035 * sqrt(3) = 0.00606  <- 正好差一个 √3
*    3. Kt 由 Kt = 1.5*pn*λ 推出, 不要手填
***********************************************************************
**/
void pmsm_2804_init(void)
{
    pm.para.pn = 7;                       /* 12N14P -> 7 对极 */

    /* 电阻/电感以「实测」为准, 不用手册值:
     *   手册写"线电阻 5.1Ω" -> 相电阻 2.55Ω
     *   万用表实测线电阻 6.2Ω (三组一致) -> 相电阻 3.1Ω
     *   差 21%, 电机个体差异 + 引线都算在里面, 实测更可信
     * 电感手册没给实测值, 按"线电感 2.8mH / 2 = 1.4mH" */
    pm.para.Rs = 6.2f / 2.0f;             /* 3.1 Ω,  实测线电阻 / 2 */
    pm.para.Ld = 2.8e-3f / 2.0f;          /* 1.4 mH,  线电感 / 2 */
    pm.para.Lq = 2.8e-3f / 2.0f;          /* 表贴式, Ld = Lq */
    pm.para.Ls = 2.8e-3f / 2.0f;
    pm.para.Ldif = 0.0f;

    pm.para.flux = 0.0062f;               /* 见上面第 2 点的换算 */

    pm.para.B  = 0.00002f;                /* 阻尼, 估的 */
    pm.para.Js = 1.0e-05f;                /* 转子惯量, 估的 (51g, 34.5mm) */

    pm.para.Gr = 1.0f;
    pm.para.ibw = 1000.0f;                /* 电流环带宽 1kHz */
    pm.para.delta = 4.0f;

    pm.para.div_pn = 1.0f / pm.para.pn;
    pm.para.pnd_2pi = pm.para.pn / M_2PI;
    pm.para.div_Gr = 1.0f / pm.para.Gr;
    pm.para.Gref = 1.0f;

    pm.para.Kt = 1.5f * pm.para.pn * pm.para.flux;   /* 0.0651 N*m/A */
    pm.para.div_Kt = 1.0f / pm.para.Kt;

    pm.ctrl.wm_acc = 200.0f;
    pm.ctrl.wm_dec = 200.0f;

    /* ⚠️ 这两个必须初始化!
     * foc_spd_pi_calc() 最后会做 pm->ctrl.iq_set = sat1_datf(iq, pmax_iq, nmax_iq),
     * 而这两个变量在 pmsm_init() 的 memset 之后一直是 0 ——
     * 结果就是速度环算出来的 iq 被夹成 0, 速度模式一点转矩都没有。
     * 上限按电机额定电流给: 2804 额定 0.5A, 留一倍余量取 1.0A。 */
    pm.ctrl.pmax_iq = 1.0f;
    pm.ctrl.nmax_iq = -1.0f;

    /* 电角度零点, 需要标定。标定前闭环不要开 */
    pm.para.e_off = 0.0f;
    pm.para.r_off = 0.0f;
    pm.para.m_off = 0.0f;
}

/**
***********************************************************************
* @brief:      pmsm_peroid_init(void)
* @param[in]:  void
* @retval:     void
* @details:    电机周期参数初始化，包括FOC、PID等相关周期和采样时间的设置
***********************************************************************
**/
void pmsm_peroid_init(void)
{
    pm.period.foc_fs = 20000.0f;
    pm.period.foc_ts = 0.00005f;

    pm.period.cur_pid_fs = 20000.0f;
    pm.period.cur_pid_ts = 1.0f/pm.period.cur_pid_fs;
    pm.period.cur_pid_cnt_val = pm.period.foc_fs * pm.period.cur_pid_ts;

    pm.period.spd_pid_fs = 10000.0f;
    pm.period.spd_pid_ts = 1.0f/pm.period.spd_pid_fs;
    pm.period.spd_pid_cnt_val = pm.period.foc_fs * pm.period.spd_pid_ts;

    pm.period.pos_pid_fs = 5000.0f;
    pm.period.pos_pid_ts = 1.0f/pm.period.pos_pid_fs;
    pm.period.pos_pid_cnt_val = pm.period.foc_fs * pm.period.pos_pid_ts;

    pm.period.spd_mea_fs = 1000.0f;
    pm.period.spd_mea_ts = 0.001f;
    pm.period.spd_mea_cnt_val = pm.period.foc_fs * pm.period.spd_mea_ts;
}

/**
***********************************************************************
* @brief:      pmsm_init(void)
* @param[in]:  void
* @retval:     void
* @details:    PMSM参数及控制器初始化，包括电机、板级、保护、周期、滤波器等参数的设置及相关初始化函数的调用
***********************************************************************
**/
void pmsm_init(void)
{
    memset(&pm, 0, sizeof(pm));
    pmsm_2804_init();

    pmsm_board_init();
    pmsm_peroid_init();
    foc_deadtime_apply();

    /* 停在 start，三相桥先不要开。输出留到进入 opera 时由状态机开一次。 */
    pm.ctrl_bit = start;

    foc_get_curr_off();

    /* 保护阈值初始化 + 电流零偏自检。
     * 必须放在 foc_get_curr_off() 之后 —— 自检要用刚采到的零偏值判断
     * VREF 有没有建立起来 (板上 U14 没焊的话这一步会置 ioff_err) */
    pmsm_protect_init();
}

/**
***********************************************************************
* @brief:      foc_spd_measure_M(float pos, float fs)
* @param[in]:  pos 当前位置（角度/弧度）
* @param[in]:  fs  采样频率
* @retval:     float 速度值
* @details:    速度测量函数，根据当前位置和采样频率计算速度
***********************************************************************
**/
_RAM_FUNC float foc_spd_measure_M(float pos, float fs)
{
    static float pos_dif;
    static float pos_last;

    pos_dif = pos - pos_last;
    wrap_pm_pi(pos_dif);

    float vel = pos_dif * fs;

    pos_last = pos;

    return vel;
}
/**
***********************************************************************
* @brief:      foc_para_calc(pmsm_t* pm)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @retval:     void
* @details:    FOC相关参数计算，包括母线电压、母线电流、滤波电流、转矩等参数的计算
***********************************************************************
**/
_RAM_FUNC void foc_para_calc(pmsm_t* pm)
{
    pm->foc.vbus = ((float)pm->adc.vbus * pm->board.v_ratio);
    pm->foc.inv_vbus = 1.5f / (pm->foc.vbus);
    pm->foc.vs = pm->foc.vbus*0.5f*0.96f;

    pm->foc.we = foc_spd_measure_M(pm->foc.p_e, 20000);
    pm->foc.wr = pm->foc.we * pm->para.div_pn; // rad/s;
}

/**
***********************************************************************
* @brief:      get_curr_off(void)
* @param[in]:  void
* @retval:     void
* @details:    电流零偏采集，采集三相ADC的偏置值并求平均，存入pm.adc结构体
***********************************************************************
**/
void foc_get_curr_off(void)
{
    /* 注意: 这里的累加变量必须显式清零。原来是 float sum_a, sum_b, sum_c;
       没给初值 —— 零偏会累加到栈上的垃圾值, 四路电流全部失真 */
    float sum_a = 0.0f;
    float sum_b = 0.0f;
    float sum_c = 0.0f;

    for (int i = 0; i < 1000; i++)
    {
        HAL_Delay(1);
        sum_a += (float)(ADC1->JDR3);
        sum_b += (float)(ADC1->JDR2);
        sum_c += (float)(ADC1->JDR1);
    }

    pm.adc.ia_off = sum_a * 0.001f;
    pm.adc.ib_off = sum_b * 0.001f;
    pm.adc.ic_off = sum_c * 0.001f;
}



/**
***********************************************************************
* @brief:      foc_pwm_start(void)
* @param[in]:  void
* @retval:     void
* @details:    启动三路PWM输出，分别对应三相电机控制
***********************************************************************
**/
static void foc_gate_pins_low(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 先写成低，再把引脚从定时器复用改成推挽输出。
     * 关断时如果只关 MOE，这几根脚会高阻，FD6288 输入悬空，三相输出被拉到母线。 */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_WritePin(GPIOA, gpio.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void foc_gate_pins_af(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_15;
    gpio.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

_RAM_FUNC void foc_pwm_start(void)
{
    TIM_TypeDef *tim = htim1.Instance;

    /* MOE 保持为 1。拉低 MOE 会让 TIM1_CC4 不再触发注入 ADC，
     * 母线电压一直是 0，20kHz 控制环停掉，三相桥永远打不开。 */
    tim->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                  TIM_CCER_CC3E | TIM_CCER_CC3NE);
    tim->BDTR |= TIM_BDTR_MOE;
    tim->CR1  |= TIM_CR1_CEN;
    foc_gate_pins_af();
}

/**
***********************************************************************
* @brief:      foc_pwm_stop(void)
* @param[in]:  void
* @retval:     void
* @details:    停止三路PWM输出，关闭三相电机控制
***********************************************************************
**/
_RAM_FUNC void foc_pwm_stop(void)
{
    /* 六路栅极脚直接拉低。不要清 MOE，也不要停 TIM1，CH4 还要触发 ADC。 */
    foc_gate_pins_low();
}

extern uint16_t adc1_buff[2];
extern uint16_t adc2_buff[4];
_RAM_FUNC void foc_adc_sample(pmsm_t* pm)
{
	pm->adc.ia = ADC1->JDR3;
	pm->adc.ib = ADC1->JDR2;
	pm->adc.ic = ADC1->JDR1;

	pm->adc.va = adc2_buff[1];
	pm->adc.vb = adc2_buff[2];
	pm->adc.vc = adc2_buff[3];
    pm->adc.vbus     = ADC2->JDR1;

    //  Convert ADC values to actual currents with offset compensation and scaling
    pm->foc.i_a = ((float) pm->adc.ia - pm->adc.ia_off) * pm->board.i_ratio;
    pm->foc.i_b = ((float) pm->adc.ib - pm->adc.ib_off) * pm->board.i_ratio;
    pm->foc.i_c = ((float) pm->adc.ic - pm->adc.ic_off) * pm->board.i_ratio;
}

/**
***********************************************************************
* @brief:      foc_pwm_run(foc_para_t* foc)
* @param[in]:  foc 指向 FOC 参数结构体的指针
* @retval:     void
* @details:    根据duty周期设置三相PWM输出，实现SVPWM调制
***********************************************************************
**/
_RAM_FUNC void foc_pwm_run(pmsm_t* pm)
{
	htim1.Instance->CCR1 = (uint16_t)(pm->foc.dtc_a * PWM_ARR());
	htim1.Instance->CCR2 = (uint16_t)(pm->foc.dtc_b * PWM_ARR());
	htim1.Instance->CCR3 = (uint16_t)(pm->foc.dtc_c * PWM_ARR());
}

_RAM_FUNC void foc_pwm_duty_set(pmsm_t* pm)
{
    htim1.Instance->CCR1 = (uint16_t)(0.5f * PWM_ARR());
    htim1.Instance->CCR2 = (uint16_t)(0.5f * PWM_ARR());
    htim1.Instance->CCR3 = (uint16_t)(0.5f * PWM_ARR());
}


/**
***********************************************************************
* @brief:      foc_volt(pmsm_t* pm, float vd_ref, float vq_ref, float pos)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @param[in]:  vd_ref d轴电压参考值
* @param[in]:  vq_ref q轴电压参考值
* @param[in]:  pos    电机电气位置（角度/弧度）
* @retval:     void
* @details:    电压控制，设置d/q轴电压参考值，完成Clarke、Park变换及SVPWM输出
***********************************************************************
**/
_RAM_FUNC void foc_volt(pmsm_t* pm, float vd_ref, float vq_ref, float pos)
{
	pm->foc.mode = foc_volt_mode;
    clarke_transform(&pm->foc);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    sin_cos_val(&pm->foc);
    park_transform(&pm->foc);
    pm->foc.v_d = vd_ref;
    pm->foc.v_q = vq_ref;
    inverse_park(&pm->foc);
    if(svm(pm->foc.v_alph * (pm->foc.inv_vbus), pm->foc.v_beta * (pm->foc.inv_vbus), &pm->foc.dtc_a, &pm->foc.dtc_b, &pm->foc.dtc_c)==0) {
        foc_pwm_run(pm);
    }
}


