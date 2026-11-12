/* Per-motor FOC state machine and current-loop implementation. */
#include "foc_motor.h"

static PwmDuty_t FocMotor_GetNeutralPwmDuty(void)
{
    PwmDuty_t pwm_duty;

    pwm_duty.phase_a = 0.5f;
    pwm_duty.phase_b = 0.5f;
    pwm_duty.phase_c = 0.5f;

    return pwm_duty;
}

static void FocMotor_ResetControlOutput(FocMotor_t *motor)
{
    FocPi_Reset(&motor->d_axis_pi);
    FocPi_Reset(&motor->q_axis_pi);

    motor->current_reference.d_A = 0.0f;
    motor->current_reference.q_A = 0.0f;

    motor->dq_voltage.d_V = 0.0f;
    motor->dq_voltage.q_V = 0.0f;

    motor->alpha_beta_voltage.alpha_V = 0.0f;
    motor->alpha_beta_voltage.beta_V = 0.0f;

    /* Keep software outputs neutral until the next control iteration. */
    motor->pwm_duty = FocMotor_GetNeutralPwmDuty();
}

void FocMotor_Init(FocMotor_t *motor, const FocMotorConfig_t *config)
{
    motor->config = *config;

    FocPi_Init(&motor->d_axis_pi, &motor->config.d_axis_pi);
    FocPi_Init(&motor->q_axis_pi, &motor->config.q_axis_pi);

    motor->feedback.phase_current.phase_a_A = 0.0f;
    motor->feedback.phase_current.phase_b_A = 0.0f;
    motor->feedback.phase_current.phase_c_A = 0.0f;
    motor->feedback.electrical_angle_rad = 0.0f;
    motor->feedback.dc_bus_voltage_V = 0.0f;

    motor->alpha_beta_current.alpha_A = 0.0f;
    motor->alpha_beta_current.beta_A = 0.0f;

    motor->dq_current.d_A = 0.0f;
    motor->dq_current.q_A = 0.0f;

    FocMotor_ResetControlOutput(motor);

    motor->state = FOC_MOTOR_STATE_STOPPED;
}

bool FocMotor_Start(FocMotor_t *motor)
{
    if (motor->state != FOC_MOTOR_STATE_STOPPED)
    {
        return false;
    }

    FocMotor_ResetControlOutput(motor);
    motor->state = FOC_MOTOR_STATE_RUNNING;

    return true;
}

void FocMotor_Stop(FocMotor_t *motor)
{
    FocMotor_ResetControlOutput(motor);

    if (motor->state != FOC_MOTOR_STATE_FAULT)
    {
        motor->state = FOC_MOTOR_STATE_STOPPED;
    }
}

void FocMotor_EnterFault(FocMotor_t *motor)
{
    FocMotor_ResetControlOutput(motor);
    motor->state = FOC_MOTOR_STATE_FAULT;
}

void FocMotor_SetCurrentReference(
    FocMotor_t *motor,
    FocCurrentReference_t current_reference)
{
    if (motor->state != FOC_MOTOR_STATE_RUNNING)
    {
        return;
    }

    motor->current_reference.d_A = FocMath_Clamp(
        current_reference.d_A,
        -motor->config.d_axis_current_limit_A,
        motor->config.d_axis_current_limit_A);

    motor->current_reference.q_A = FocMath_Clamp(
        current_reference.q_A,
        -motor->config.q_axis_current_limit_A,
        motor->config.q_axis_current_limit_A);
}

PwmDuty_t FocMotor_RunCurrentLoop(
    FocMotor_t *motor,
    const FocMotorFeedback_t *feedback)
{
    FocSinCos_t sin_cos;

    if (motor->state != FOC_MOTOR_STATE_RUNNING)
    {
        motor->pwm_duty = FocMotor_GetNeutralPwmDuty();

        return motor->pwm_duty;
    }

    motor->feedback = *feedback;

    motor->alpha_beta_current = FocMath_Clarke(
        motor->feedback.phase_current);

    sin_cos = FocMath_GetSinCos(
        motor->feedback.electrical_angle_rad);

    motor->dq_current = FocMath_Park(
        motor->alpha_beta_current,
        sin_cos);

    motor->dq_voltage.d_V = FocPi_Update(
        &motor->d_axis_pi,
        motor->current_reference.d_A,
        motor->dq_current.d_A,
        motor->config.current_loop_period_s);

    motor->dq_voltage.q_V = FocPi_Update(
        &motor->q_axis_pi,
        motor->current_reference.q_A,
        motor->dq_current.q_A,
        motor->config.current_loop_period_s);

    motor->alpha_beta_voltage = FocMath_InversePark(
        motor->dq_voltage,
        sin_cos);

    motor->pwm_duty = FocMath_Svpwm(
        motor->alpha_beta_voltage,
        motor->feedback.dc_bus_voltage_V);

    return motor->pwm_duty;
}
