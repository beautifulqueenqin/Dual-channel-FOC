/* Dual-motor coordination and shared-policy implementation. */
#include "dual_motor.h"
#include <stddef.h>

static bool DualMotor_IsValidMotorId(MotorId_t motor_id);
static DualMotorPwmOutput_t DualMotor_GetNeutralPwmOutput(void);

bool DualMotor_Init(
    DualMotor_t *dual_motor,
    const DualMotorConfig_t *config)
{
    MotorId_t motor_id;

    if ((dual_motor == NULL) || (config == NULL))
    {
        return false;
    }

    /*
     * Validate every configuration before modifying runtime data.
     * The array index must match the configured motor ID.
     */
    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        const MotorId_t configured_motor_id =
            config->motor_config[motor_id].motor_id;

        if ((!DualMotor_IsValidMotorId(configured_motor_id)) ||
            (configured_motor_id != motor_id))
        {
            return false;
        }
    }

    /* Initialize both independent motor-control instances. */
    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        FocMotor_Init(
            &dual_motor->motor[motor_id],
            &config->motor_config[motor_id]);
    }

    dual_motor->fault_latched = false;

    return true;
}

bool DualMotor_StartMotor(
    DualMotor_t *dual_motor,
    MotorId_t motor_id)
{
    if (dual_motor == NULL)
    {
        return false;
    }

    if (!DualMotor_IsValidMotorId(motor_id))
    {
        return false;
    }

    /*
     * No motor may start while a shared fault is latched.
     */
    if (dual_motor->fault_latched)
    {
        return false;
    }

    return FocMotor_Start(
        &dual_motor->motor[motor_id]);
}

bool DualMotor_StopMotor(
    DualMotor_t *dual_motor,
    MotorId_t motor_id)
{
    if (dual_motor == NULL)
    {
        return false;
    }

    if (!DualMotor_IsValidMotorId(motor_id))
    {
        return false;
    }

    FocMotor_Stop(
        &dual_motor->motor[motor_id]);

    /*
     * A faulted motor remains faulted and therefore reports failure.
     */
    return (dual_motor->motor[motor_id].state ==
            FOC_MOTOR_STATE_STOPPED);
}

void DualMotor_StopAll(DualMotor_t *dual_motor)
{
    MotorId_t motor_id;

    if (dual_motor == NULL)
    {
        return;
    }

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        (void)DualMotor_StopMotor(
            dual_motor,
            motor_id);
    }
}

bool DualMotor_SetCurrentReference(
    DualMotor_t *dual_motor,
    MotorId_t motor_id,
    FocCurrentReference_t current_reference)
{
    if (dual_motor == NULL)
    {
        return false;
    }

    if (!DualMotor_IsValidMotorId(motor_id))
    {
        return false;
    }

    /*
     * No current command may be accepted while a shared
     * fault is latched.
     */
    if (dual_motor->fault_latched)
    {
        return false;
    }

    if (dual_motor->motor[motor_id].state !=
        FOC_MOTOR_STATE_RUNNING)
    {
        return false;
    }

    FocMotor_SetCurrentReference(
        &dual_motor->motor[motor_id],
        current_reference);

    return true;
}

void DualMotor_EnterFault(DualMotor_t *dual_motor)
{
    MotorId_t motor_id;

    if (dual_motor == NULL)
    {
        return;
    }

    /*
     * Latch the shared fault before changing individual motor states.
     */
    dual_motor->fault_latched = true;

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        FocMotor_EnterFault(
            &dual_motor->motor[motor_id]);
    }
}

bool DualMotor_IsFaultLatched(
    const DualMotor_t *dual_motor)
{
    if (dual_motor == NULL)
    {
        /*
         * An unknown controller must be treated as unsafe.
         */
        return true;
    }

    return dual_motor->fault_latched;
}

bool DualMotor_ClearFault(DualMotor_t *dual_motor)
{
    MotorId_t motor_id;

    if (dual_motor == NULL)
    {
        return false;
    }

    if (!dual_motor->fault_latched)
    {
        return false;
    }

    /*
     * Keep the global fault latched while both motors are reset.
     */
    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        const FocMotorConfig_t motor_config =
            dual_motor->motor[motor_id].config;

        FocMotor_Init(
            &dual_motor->motor[motor_id],
            &motor_config);
    }

    /*
     * Release the global fault only after every motor is safe.
     */
    dual_motor->fault_latched = false;

    return true;
}

DualMotorPwmOutput_t DualMotor_RunCurrentLoop(
    DualMotor_t *dual_motor,
    const DualMotorFeedback_t *feedback)
{
    DualMotorPwmOutput_t pwm_output;
    MotorId_t motor_id;

    if ((dual_motor == NULL) ||
        (feedback == NULL) ||
        dual_motor->fault_latched)
    {
        return DualMotor_GetNeutralPwmOutput();
    }

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        pwm_output.pwm_duty[motor_id] =
            FocMotor_RunCurrentLoop(
                &dual_motor->motor[motor_id],
                &feedback->motor_feedback[motor_id]);
    }

    return pwm_output;
}

/**
 * @brief Checks whether a motor ID can safely index motor arrays.
 */
static bool DualMotor_IsValidMotorId(MotorId_t motor_id)
{
    return ((unsigned int)motor_id <
            (unsigned int)MOTOR_ID_COUNT);
}

/**
 * @brief Returns neutral PWM duty commands for every motor channel.
 */
static DualMotorPwmOutput_t DualMotor_GetNeutralPwmOutput(void)
{
    DualMotorPwmOutput_t pwm_output;
    MotorId_t motor_id;

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        pwm_output.pwm_duty[motor_id].phase_a = 0.5f;
        pwm_output.pwm_duty[motor_id].phase_b = 0.5f;
        pwm_output.pwm_duty[motor_id].phase_c = 0.5f;
    }

    return pwm_output;
}
