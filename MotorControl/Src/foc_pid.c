/* FOC PI-controller implementation. */
#include "foc_pid.h"
#include <stdbool.h>

static float FocPi_Clamp(float value, float lower_limit, float upper_limit);

void FocPi_Reset(FocPi_t *controller)
{
    controller->integrator = 0.0f;
    controller->last_output = 0.0f;
}

void FocPi_Init(FocPi_t *controller, const FocPiConfig_t *config)
{
    controller->config = *config;

    FocPi_Reset(controller);
}

float FocPi_Update(FocPi_t *controller,
                   float reference,
                   float feedback,
                   float sample_time_s)
{
    const float error = reference - feedback;
    const float proportional = controller->config.kp * error;
    const float integrator_candidate =
        controller->integrator +
        (controller->config.ki * error * sample_time_s);
    const float output_candidate = proportional + integrator_candidate;

    const bool is_pushing_upper_limit =
        (output_candidate > controller->config.output_max) &&
        (error > 0.0f);

    const bool is_pushing_lower_limit =
        (output_candidate < controller->config.output_min) &&
        (error < 0.0f);

    /* Do not integrate further into a saturated output. */
    if ((!is_pushing_upper_limit) && (!is_pushing_lower_limit))
    {
        controller->integrator = integrator_candidate;
    }

    controller->last_output = FocPi_Clamp(
        proportional + controller->integrator,
        controller->config.output_min,
        controller->config.output_max);

    return controller->last_output;
}

static float FocPi_Clamp(float value, float lower_limit, float upper_limit)
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
