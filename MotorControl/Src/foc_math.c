/* FOC pure-math implementation. */
#include "foc_math.h"
#include <math.h>


static float FocMath_Max3(float first, float second, float third);
static float FocMath_Min3(float first, float second, float third);

static const float FOC_MATH_TWO_THIRDS = 0.6666666667f;
static const float FOC_MATH_INV_SQRT_THREE = 0.5773502692f;
static const float FOC_MATH_SQRT_THREE_OVER_TWO = 0.8660254038f;



float FocMath_Clamp(float value, float lower_limit, float upper_limit)
{
    if (value > upper_limit)
    {
        return upper_limit;
    }

    if (value < lower_limit)
    {
        return lower_limit;
    }

    return value;
}

AlphaBetaCurrent_t FocMath_Clarke(PhaseCurrent_t phase_current)
{
    AlphaBetaCurrent_t alpha_beta_current;

    alpha_beta_current.alpha_A =
        FOC_MATH_TWO_THIRDS *
        (phase_current.phase_a_A -
         (0.5f * phase_current.phase_b_A) -
         (0.5f * phase_current.phase_c_A));

    alpha_beta_current.beta_A =
        FOC_MATH_INV_SQRT_THREE *
        (phase_current.phase_b_A - phase_current.phase_c_A);

    return alpha_beta_current;
}

FocSinCos_t FocMath_GetSinCos(float electrical_angle_rad)
{
    FocSinCos_t sin_cos;

    sin_cos.sin_theta = sinf(electrical_angle_rad);
    sin_cos.cos_theta = cosf(electrical_angle_rad);

    return sin_cos;
}

DqCurrent_t FocMath_Park(AlphaBetaCurrent_t alpha_beta_current,
                          FocSinCos_t sin_cos)
{
    DqCurrent_t dq_current;

    dq_current.d_A =
        (alpha_beta_current.alpha_A * sin_cos.cos_theta) +
        (alpha_beta_current.beta_A * sin_cos.sin_theta);

    dq_current.q_A =
        (-alpha_beta_current.alpha_A * sin_cos.sin_theta) +
        (alpha_beta_current.beta_A * sin_cos.cos_theta);

    return dq_current;
}

AlphaBetaVoltage_t FocMath_InversePark(DqVoltage_t dq_voltage,
                                        FocSinCos_t sin_cos)
{
    AlphaBetaVoltage_t alpha_beta_voltage;

    alpha_beta_voltage.alpha_V =
        (dq_voltage.d_V * sin_cos.cos_theta) -
        (dq_voltage.q_V * sin_cos.sin_theta);

    alpha_beta_voltage.beta_V =
        (dq_voltage.d_V * sin_cos.sin_theta) +
        (dq_voltage.q_V * sin_cos.cos_theta);

    return alpha_beta_voltage;
}

PwmDuty_t FocMath_Svpwm(AlphaBetaVoltage_t alpha_beta_voltage,
                         float dc_bus_voltage_V)
{
    PwmDuty_t pwm_duty;
    float phase_a_voltage;
    float phase_b_voltage;
    float phase_c_voltage;
    float maximum_phase_voltage;
    float minimum_phase_voltage;
    float voltage_span;
    float voltage_scale;
    float common_mode_voltage;

    if (dc_bus_voltage_V <= 0.0f)
    {
        pwm_duty.phase_a = 0.5f;
        pwm_duty.phase_b = 0.5f;
        pwm_duty.phase_c = 0.5f;

        return pwm_duty;
    }

    phase_a_voltage = alpha_beta_voltage.alpha_V;

    phase_b_voltage =
        (-0.5f * alpha_beta_voltage.alpha_V) +
        (FOC_MATH_SQRT_THREE_OVER_TWO * alpha_beta_voltage.beta_V);

    phase_c_voltage =
        (-0.5f * alpha_beta_voltage.alpha_V) -
        (FOC_MATH_SQRT_THREE_OVER_TWO * alpha_beta_voltage.beta_V);

    maximum_phase_voltage = FocMath_Max3(
        phase_a_voltage,
        phase_b_voltage,
        phase_c_voltage);

    minimum_phase_voltage = FocMath_Min3(
        phase_a_voltage,
        phase_b_voltage,
        phase_c_voltage);

    voltage_span = maximum_phase_voltage - minimum_phase_voltage;
    voltage_scale = 1.0f;

    if (voltage_span > dc_bus_voltage_V)
    {
        voltage_scale = dc_bus_voltage_V / voltage_span;

        phase_a_voltage *= voltage_scale;
        phase_b_voltage *= voltage_scale;
        phase_c_voltage *= voltage_scale;

        maximum_phase_voltage = FocMath_Max3(
            phase_a_voltage,
            phase_b_voltage,
            phase_c_voltage);

        minimum_phase_voltage = FocMath_Min3(
            phase_a_voltage,
            phase_b_voltage,
            phase_c_voltage);
    }

    common_mode_voltage =
        -0.5f * (maximum_phase_voltage + minimum_phase_voltage);

    phase_a_voltage += common_mode_voltage;
    phase_b_voltage += common_mode_voltage;
    phase_c_voltage += common_mode_voltage;

    pwm_duty.phase_a = FocMath_Clamp(
        0.5f + (phase_a_voltage / dc_bus_voltage_V),
        0.0f,
        1.0f);

    pwm_duty.phase_b = FocMath_Clamp(
        0.5f + (phase_b_voltage / dc_bus_voltage_V),
        0.0f,
        1.0f);

    pwm_duty.phase_c = FocMath_Clamp(
        0.5f + (phase_c_voltage / dc_bus_voltage_V),
        0.0f,
        1.0f);

    return pwm_duty;
}

static float FocMath_Max3(float first, float second, float third)
{
    float maximum = first;

    if (second > maximum)
    {
        maximum = second;
    }

    if (third > maximum)
    {
        maximum = third;
    }

    return maximum;
}

static float FocMath_Min3(float first, float second, float third)
{
    float minimum = first;

    if (second < minimum)
    {
        minimum = second;
    }

    if (third < minimum)
    {
        minimum = third;
    }

    return minimum;
}
