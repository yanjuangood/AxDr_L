#include "common.h"

/**
***********************************************************************
* @brief:      foc_calc(pmsm_foc_t *foc)
* @param[in]:  foc  指向磁场定向控制参数结构体的指针
* @retval:     void
* @details:    一次完整计算：正余弦、克拉克、帕克、逆帕克，再做空间矢量调制
***********************************************************************
**/
_RAM_FUNC void foc_calc(pmsm_foc_t *foc)
{
    foc->sin_val = sinf(foc->theta);
    foc->cos_val = cosf(foc->theta);
    foc->i_alph = foc->i_a;
    foc->i_beta = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
    foc->i_d = foc->i_alph * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q = foc->i_beta * foc->cos_val - foc->i_alph * foc->sin_val;
    foc->v_alph = (foc->v_d *foc->cos_val - foc->v_q *foc->sin_val);
    foc->v_beta = (foc->v_d *foc->sin_val + foc->v_q *foc->cos_val);
    svm(foc->v_alph*(foc->inv_vbus), foc->v_beta*(foc->inv_vbus), &foc->dtc_a, &foc->dtc_b, &foc->dtc_c);
}

/**
***********************************************************************
* @brief:      sin_cos_val(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    计算角度 theta 对应的 sin 和 cos 值
***********************************************************************
**/
_RAM_FUNC void sin_cos_val(pmsm_foc_t *foc)
{
    foc->sin_val = sinf(foc->theta);
    foc->cos_val = cosf(foc->theta);
}

/**
***********************************************************************
* @brief:      clarke_transform(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    Clarke变换，将三相电流变换为Alpha-Beta坐标系下的电流
***********************************************************************
**/
_RAM_FUNC void clarke_transform(pmsm_foc_t *foc)
{
    foc->i_alph = foc->i_a;
    foc->i_beta = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
}

/**
***********************************************************************
* @brief:      inverse_clarke(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    逆Clarke变换，将 Alpha-Beta 坐标系下的电压转换为三相电压
***********************************************************************
**/
_RAM_FUNC void inverse_clarke(pmsm_foc_t *foc)
{
    foc->v_a = foc->v_alph;
    foc->v_b = -0.5f * foc->v_alph + SQRT3_BY_2 *foc->v_beta;
    foc->v_c = -0.5f * foc->v_alph - SQRT3_BY_2 *foc->v_beta;
}

/**
***********************************************************************
* @brief:      park_transform(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    Park变换，将 Alpha-Beta 坐标系下的电流转换为 dq 坐标系下的电流
***********************************************************************
**/
_RAM_FUNC void park_transform(pmsm_foc_t *foc)
{
    foc->i_d = foc->i_alph * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q = foc->i_beta * foc->cos_val - foc->i_alph * foc->sin_val;
}

/**
***********************************************************************
* @brief:      inverse_park(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    逆Park变换，将 dq 坐标系下的电压转换为 Alpha-Beta 坐标系下的电压
***********************************************************************
**/
_RAM_FUNC void inverse_park(pmsm_foc_t *foc)
{
    foc->v_alph = (foc->v_d *foc->cos_val - foc->v_q *foc->sin_val);
    foc->v_beta = (foc->v_d *foc->sin_val + foc->v_q *foc->cos_val);
}

/**
***********************************************************************
* @brief:      svpwm_midpoint(pmsm_foc_t *foc)
* @param[in]:  foc  指向 FOC 参数结构体的指针
* @retval:     void
* @details:    中点钳位空间矢量调制。用共模电压把三相占空比收进 0 到 1
***********************************************************************
**/
_RAM_FUNC void svpwm_midpoint(pmsm_foc_t *foc)
{
    foc->v_alph = foc->inv_vbus *foc->v_alph;
    foc->v_beta = foc->inv_vbus *foc->v_beta;
    float va = foc->v_alph;
    float vb = -0.5f * foc->v_alph + SQRT3_BY_2 *foc->v_beta;
    float vc = -0.5f * foc->v_alph - SQRT3_BY_2 *foc->v_beta;
    float vmax = max(max(va, vb), vc);
    float vmin = min(min(va, vb), vc);
    float vcom = (vmax + vmin) * 0.5f;
    foc->dtc_a = 1.0f-((va - vcom) + 0.5f);
    foc->dtc_b = 1.0f-((vb - vcom) + 0.5f);
    foc->dtc_c = 1.0f-((vc - vcom) + 0.5f);
}

/**
***********************************************************************
* @brief:      svpwm_sector(pmsm_foc_t *foc)
* @param[in]:  foc  指向 FOC 参数结构体的指针
* @retval:     void
* @details:    扇区法空间矢量调制。按两相静止坐标所在扇区计算矢量作用时间，再写成占空比
***********************************************************************
**/
_RAM_FUNC void svpwm_sector(pmsm_foc_t *foc)
{
    float TS = 1.0f;
    float ta = 0.0f, tb = 0.0f, tc = 0.0f;
    float k = (TS *SQRT3) * foc->inv_vbus;
    float va = foc->v_beta;
    float vb = (SQRT3 *foc->v_alph - foc->v_beta) * 0.5f;
    float vc = (-SQRT3 *foc->v_alph - foc->v_beta) * 0.5f;
    int a = (va > 0.0f) ? 1 : 0;
    int b = (vb > 0.0f) ? 1 : 0;
    int c = (vc > 0.0f) ? 1 : 0;
    int sextant = (c << 2) + (b << 1) + a;

    switch (sextant)
    {
    case 3:
    {
        float t4 = k *vb;
        float t6 = k *va;
        float t0 = (TS - t4 - t6) * 0.5f;
        ta = t4 + t6 + t0;
        tb = t6 + t0;
        tc = t0;
    }
    break;

    case 1:
    {
        float t6 = -k *vc;
        float t2 = -k *vb;
        float t0 = (TS - t2 - t6) * 0.5f;
        ta = t6 + t0;
        tb = t2 + t6 + t0;
        tc = t0;
    }
    break;

    case 5:
    {
        float t2 = k *va;
        float t3 = k *vc;
        float t0 = (TS - t2 - t3) * 0.5f;
        ta = t0;
        tb = t2 + t3 + t0;
        tc = t3 + t0;
    }
    break;

    case 4:
    {
        float t1 = -k *va;
        float t3 = -k *vb;
        float t0 = (TS - t1 - t3) * 0.5f;
        ta = t0;
        tb = t3 + t0;
        tc = t1 + t3 + t0;
    }
    break;

    case 6:
    {
        float t1 = k *vc;
        float t5 = k *vb;
        float t0 = (TS - t1 - t5) * 0.5f;
        ta = t5 + t0;
        tb = t0;
        tc = t1 + t5 + t0;
    }
    break;

    case 2:
    {
        float t4 = -k *vc;
        float t5 = -k *va;
        float t0 = (TS - t4 - t5) * 0.5f;
        ta = t4 + t5 + t0;
        tb = t0;
        tc = t5 + t0;
    }
    break;

    default:
        break;
    }

    foc->dtc_a = 1.0f - ta;
    foc->dtc_b = 1.0f - tb;
    foc->dtc_c = 1.0f - tc;
}

/* ===========================================================================
 * ⚠️⚠️ svm() 的符号约定和 svpwm() 不一致, 用之前必须知道这件事 ⚠️⚠️
 *
 * 手算验证 (alpha=+0.866, beta=0, 落到 Sextant 1):
 *      t1 = 0.866, t2 = 0
 *      tA = (1-0.866-0)*0.5 = 0.067
 *      tB = 0.933,  tC = 0.933
 *   占空比 0.067/0.933/0.933 -> 相对中性点的相电压 (标幺, 减掉共模 0.644):
 *      vA = -0.577,  vB = +0.289,  vC = +0.289
 *   再克拉克正算:  alpha = (2*vA - vB - vC)/3 = -0.577
 *   而请求的是 alpha = +0.577
 *
 *   ⇒ 输出的电压矢量是「请求值的相反数」。
 *   对比 svpwm(): 它在最后做了 foc->dtc_x = 1.0f - t_x, svm() 里没有这一步,
 *     两个函数的约定不一致, 这就是根源。
 *
 * 影响:
 *   - 开环 V/f: 只是磁场反向旋转。电机会朝相反方向转, 功能上没问题,
 *     想改方向把电机任意两根线对调即可。
 *   - 电流闭环 (foc_curr / foc_vel): 如果电流反馈那条链是标准符号,
 *     这个反相就变成【正反馈】—— 电压会一路顶到限幅, 直接过流。
 *     ⚠️ 在启用电流环之前必须先确认这一点, 不能直接上电跑闭环。
 *
 * 为什么现在不改: 一改开环的旋转方向就反了, 需要重新验证;
 *   而且闭环的另一半 (电流采样极性) 因为 U14 没焊, 现在根本测不了。
 *   等 U14 焊上、能测电流了, 再一次性把整条链的符号对清楚。
 * =========================================================================== */
/**
***********************************************************************
* @brief:      svm(float alpha, float beta, float *ta, float *tb, float *tc)
* @param[in]:  alpha  标幺后的 α 轴电压
* @param[in]:  beta   标幺后的 β 轴电压
* @param[out]: ta/tb/tc  三相占空比
* @retval:     0 占空比都在 0 到 1；-1 越界或出现非数，调用方应保持上一拍输出
* @details:    六扇区空间矢量调制。符号约定见上面的说明，输出矢量与请求值相反
***********************************************************************
**/
int svm(float alpha, float beta, float *ta, float *tb, float *tc)
{
    int Sextant;
	float tA, tB, tC;
    if (beta >= 0.0f)
    {
        if (alpha >= 0.0f)
        {
            /* 第一象限 */
            if (ONE_BY_SQRT3 *beta > alpha)
            {
                Sextant = 2;    /* 扇区 2，矢量 V2-V3 */
            }
            else
            {
                Sextant = 1;    /* 扇区 1，矢量 V1-V2 */
            }
        }
        else
        {
            /* 第二象限 */
            if (-ONE_BY_SQRT3 *beta > alpha)
            {
                Sextant = 3;    /* 扇区 3，矢量 V3-V4 */
            }
            else
            {
                Sextant = 2;    /* 扇区 2，矢量 V2-V3 */
            }
        }
    }
    else
    {
        if (alpha >= 0.0f)
        {
            /* 第四象限 */
            if (-ONE_BY_SQRT3 *beta > alpha)
            {
                Sextant = 5;    /* 扇区 5，矢量 V5-V6 */
            }
            else
            {
                Sextant = 6;    /* 扇区 6，矢量 V6-V1 */
            }
        }
        else
        {
            /* 第三象限 */
            if (ONE_BY_SQRT3 *beta > alpha)
            {
                Sextant = 4;    /* 扇区 4，矢量 V4-V5 */
            }
            else
            {
                Sextant = 5;    /* 扇区 5，矢量 V5-V6 */
            }
        }
    }

    switch (Sextant)
    {
    /* 扇区 1，矢量 V1 到 V2 */
    case 1:
    {
        /* 相邻两个基本矢量的作用时间 */
        float t1 = alpha - ONE_BY_SQRT3 *beta;
        float t2 = TWO_BY_SQRT3 *beta;
        /* 换算成三相开通时刻 */
        tA = (1.0f - t1 - t2) * 0.5f;
        tB = tA + t1;
        tC = tB + t2;
    }
    break;

    /* 扇区 2，矢量 V2 到 V3 */
    case 2:
    {
        /* 相邻两个基本矢量的作用时间 */
        float t2 = alpha + ONE_BY_SQRT3 *beta;
        float t3 = -alpha + ONE_BY_SQRT3 *beta;
        /* 换算成三相开通时刻 */
        tB = (1.0f - t2 - t3) * 0.5f;
        tA = tB + t3;
        tC = tA + t2;
    }
    break;

    /* 扇区 3，矢量 V3 到 V4 */
    case 3:
    {
        /* 相邻两个基本矢量的作用时间 */
        float t3 = TWO_BY_SQRT3 *beta;
        float t4 = -alpha - ONE_BY_SQRT3 *beta;
        /* 换算成三相开通时刻 */
        tB = (1.0f - t3 - t4) * 0.5f;
        tC = tB + t3;
        tA = tC + t4;
    }
    break;

    /* 扇区 4，矢量 V4 到 V5 */
    case 4:
    {
        /* 相邻两个基本矢量的作用时间 */
        float t4 = -alpha + ONE_BY_SQRT3 *beta;
        float t5 = -TWO_BY_SQRT3 *beta;
        /* 换算成三相开通时刻 */
        tC = (1.0f - t4 - t5) * 0.5f;
        tB = tC + t5;
        tA = tB + t4;
    }
    break;

    /* 扇区 5，矢量 V5 到 V6 */
    case 5:
    {
        /* 相邻两个基本矢量的作用时间 */
        float t5 = -alpha - ONE_BY_SQRT3 *beta;
        float t6 = alpha - ONE_BY_SQRT3 *beta;
        /* 换算成三相开通时刻 */
        tC = (1.0f - t5 - t6) * 0.5f;
        tA = tC + t5;
        tB = tA + t6;
    }
    break;

    /* 扇区 6，矢量 V6 到 V1 */
    case 6:
    {
        /* 相邻两个基本矢量的作用时间 */
        float t6 = -TWO_BY_SQRT3 *beta;
        float t1 = alpha + ONE_BY_SQRT3 *beta;
        /* 换算成三相开通时刻 */
        tA = (1.0f - t6 - t1) * 0.5f;
        tC = tA + t1;
        tB = tC + t6;
    }
    break;
    }
	
    *ta = tA;
    *tb = tB;
    *tc = tC;

    int result_valid = *ta >= 0.0f && *ta <= 1.0f && *tb >= 0.0f && *tb <= 1.0f && *tc >= 0.0f && *tc <= 1.0f;
    /* 任一结果变成非数时，下面的比较结果为假 */
   
    return result_valid ? 0 : -1;
}
