#ifndef DUAL_MOTOR_H_
#define DUAL_MOTOR_H_

#include <stdbool.h>

#include "foc_motor.h"

/**
 * @brief Fixed configuration for the complete dual-motor controller.
 */
typedef struct
{
    FocMotorConfig_t motor_config[MOTOR_ID_COUNT];
} DualMotorConfig_t;

/**
 * @brief Feedback data required by both motor-control instances.
 */
typedef struct
{
    FocMotorFeedback_t motor_feedback[MOTOR_ID_COUNT];
} DualMotorFeedback_t;

/**
 * @brief PWM duty outputs produced for both motors.
 */
typedef struct
{
    PwmDuty_t pwm_duty[MOTOR_ID_COUNT];
} DualMotorPwmOutput_t;

/**
 * @brief Runtime data for the complete dual-motor controller.
 */
typedef struct
{
    FocMotor_t motor[MOTOR_ID_COUNT];

    /*
     * A shared fault stops both motors and remains active until
     * explicitly cleared by a safe upper-level procedure.
     */
    bool fault_latched;
} DualMotor_t;

/**
 * @brief Initializes both motor-control channels.
 *
 * @return True when both channel configurations are valid.
 * @note This function does not enable PWM or gate-driver hardware.
 */
bool DualMotor_Init(
    DualMotor_t *dual_motor,
    const DualMotorConfig_t *config);

/**
 * @brief Starts one motor-control channel.
 *
 * @return True when the selected motor enters the running state.
 */
bool DualMotor_StartMotor(
    DualMotor_t *dual_motor,
    MotorId_t motor_id);

/**
 * @brief Stops one motor-control channel and resets its PI state.
 *
 * @return True when the selected motor enters the stopped state.
 */
bool DualMotor_StopMotor(
    DualMotor_t *dual_motor,
    MotorId_t motor_id);

/**
 * @brief Stops every motor-control channel.
 *
 * @note A faulted channel remains in the fault state.
 */
void DualMotor_StopAll(DualMotor_t *dual_motor);

/**
 * @brief Updates the current reference of one running motor.
 *
 * @return True when the reference is accepted.
 */
bool DualMotor_SetCurrentReference(
    DualMotor_t *dual_motor,
    MotorId_t motor_id,
    FocCurrentReference_t current_reference);

/**
 * @brief Latches a global software fault and faults both motors.
 *
 * @note Immediate hardware shutdown remains the responsibility
 *       of the BSP safety layer.
 */
void DualMotor_EnterFault(DualMotor_t *dual_motor);

/**
 * @brief Clears the global software fault and resets both motors.
 *
 * @return True when a latched fault is successfully cleared.
 * @note The caller must first verify that the physical fault has
 *       disappeared and that the power stage is safely disabled.
 */
bool DualMotor_ClearFault(DualMotor_t *dual_motor);

/**
 * @brief Reports whether the global software fault is latched.
 */
bool DualMotor_IsFaultLatched(
    const DualMotor_t *dual_motor);

/**
 * @brief Executes one current-loop iteration for both motors.
 *
 * @return PWM duty outputs indexed by MotorId_t.
 * @note A stopped or faulted motor produces neutral PWM duty.
 */
DualMotorPwmOutput_t DualMotor_RunCurrentLoop(
    DualMotor_t *dual_motor,
    const DualMotorFeedback_t *feedback);

#endif /* DUAL_MOTOR_H_ */
