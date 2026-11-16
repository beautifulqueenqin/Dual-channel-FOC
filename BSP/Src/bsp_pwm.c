/* TIM1/TIM8 PWM hardware adaptation. */
#include "bsp_pwm.h"

#include <stddef.h>

#include "tim.h"

static TIM_HandleTypeDef *BspPwm_GetTimer(
    MotorId_t motor_id);

static uint32_t BspPwm_DutyToCompare(
    float duty,
    uint32_t period);

static bool BspPwm_StartTimer(
    TIM_HandleTypeDef *timer);

static void BspPwm_StopTimer(
    TIM_HandleTypeDef *timer);

void BspPwm_Init(void)
{
    const PwmDuty_t neutral_duty =
        {
            .phase_a = 0.5f,
            .phase_b = 0.5f,
            .phase_c = 0.5f};

    /*
     * Keep all PWM outputs disabled during initialization.
     */
    BspPwm_Stop();

    /*
     * Center all three phases before enabling PWM.
     */
    (void)BspPwm_SetMotorDuty(
        MOTOR_ID_0,
        &neutral_duty);

    (void)BspPwm_SetMotorDuty(
        MOTOR_ID_1,
        &neutral_duty);
}

bool BspPwm_Start(void)
{
    if (!BspPwm_StartTimer(&htim1))
    {
        BspPwm_Stop();
        return false;
    }

    if (!BspPwm_StartTimer(&htim8))
    {
        BspPwm_Stop();
        return false;
    }

    return true;
}

void BspPwm_Stop(void)
{
    BspPwm_StopTimer(&htim1);
    BspPwm_StopTimer(&htim8);
}

bool BspPwm_SetMotorDuty(
    MotorId_t motor_id,
    const PwmDuty_t *pwm_duty)
{
    TIM_HandleTypeDef *timer;
    uint32_t period;

    if (pwm_duty == NULL)
    {
        return false;
    }

    timer = BspPwm_GetTimer(motor_id);

    if (timer == NULL)
    {
        return false;
    }

    period = __HAL_TIM_GET_AUTORELOAD(timer);

    __HAL_TIM_SET_COMPARE(
        timer,
        TIM_CHANNEL_1,
        BspPwm_DutyToCompare(
            pwm_duty->phase_a,
            period));

    __HAL_TIM_SET_COMPARE(
        timer,
        TIM_CHANNEL_2,
        BspPwm_DutyToCompare(
            pwm_duty->phase_b,
            period));

    __HAL_TIM_SET_COMPARE(
        timer,
        TIM_CHANNEL_3,
        BspPwm_DutyToCompare(
            pwm_duty->phase_c,
            period));

    return true;
}

static TIM_HandleTypeDef *BspPwm_GetTimer(
    MotorId_t motor_id)
{
    switch (motor_id)
    {
    case MOTOR_ID_0:
        return &htim1;

    case MOTOR_ID_1:
        return &htim8;

    default:
        return NULL;
    }
}

static uint32_t BspPwm_DutyToCompare(
    float duty,
    uint32_t period)
{
    /*
     * This condition also rejects NaN values.
     */
    if (!(duty > 0.0f))
    {
        return 0U;
    }

    if (duty >= 1.0f)
    {
        return period;
    }

    return (uint32_t)((duty * (float)period) + 0.5f);
}

static bool BspPwm_StartTimer(
    TIM_HandleTypeDef *timer)
{
    if (HAL_TIM_PWM_Start(
            timer,
            TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Start(
            timer,
            TIM_CHANNEL_2) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Start(
            timer,
            TIM_CHANNEL_3) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIMEx_PWMN_Start(
            timer,
            TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIMEx_PWMN_Start(
            timer,
            TIM_CHANNEL_2) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIMEx_PWMN_Start(
            timer,
            TIM_CHANNEL_3) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_OC_Start(timer, TIM_CHANNEL_4) != HAL_OK)
    {
        return false;
    }

    return true;
}

static void BspPwm_StopTimer(
    TIM_HandleTypeDef *timer)
{
    (void)HAL_TIM_PWM_Stop(
        timer,
        TIM_CHANNEL_1);

    (void)HAL_TIM_PWM_Stop(
        timer,
        TIM_CHANNEL_2);

    (void)HAL_TIM_PWM_Stop(
        timer,
        TIM_CHANNEL_3);

    (void)HAL_TIMEx_PWMN_Stop(
        timer,
        TIM_CHANNEL_1);

    (void)HAL_TIMEx_PWMN_Stop(
        timer,
        TIM_CHANNEL_2);

    (void)HAL_TIMEx_PWMN_Stop(
        timer,
        TIM_CHANNEL_3);

    (void)HAL_TIM_OC_Stop(
        timer,
        TIM_CHANNEL_4);
}