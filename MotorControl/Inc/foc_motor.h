#ifndef FOC_MOTOR_H_
#define FOC_MOTOR_H_

#include "foc_math.h"
#include "foc_pid.h"
#include <stdbool.h>

/**
 * @brief Fixed configuration for one FOC motor control instance.
 */
typedef struct
{
    MotorId_t motor_id;

    /* Current-loop execution period in seconds. */
    float current_loop_period_s;
    /* Absolute d-axis current command limit in ampere. */
    float d_axis_current_limit_A;
    /* Absolute q-axis current command limit in ampere. */
    float q_axis_current_limit_A;

    FocPiConfig_t d_axis_pi;
    FocPiConfig_t q_axis_pi;
} FocMotorConfig_t;

/**
 * @brief Measurements required for one FOC current-control iteration.
 */
typedef struct
{
    PhaseCurrent_t phase_current;

    /* Electrical rotor angle in radians. */
    float electrical_angle_rad;

    /* Measured DC-link voltage in volt. */
    float dc_bus_voltage_V;
} FocMotorFeedback_t;

/**
 * @brief d-q current command for one FOC motor.
 */
typedef struct
{
    float d_A;
    float q_A;
} FocCurrentReference_t;

/**
 * @brief Software state of one FOC motor control instance.
 */
typedef enum
{
    FOC_MOTOR_STATE_UNINITIALIZED = 0,
    FOC_MOTOR_STATE_STOPPED,
    FOC_MOTOR_STATE_RUNNING,
    FOC_MOTOR_STATE_FAULT
} FocMotorState_t;

/**
 * @brief Runtime data for one independent FOC motor control instance.
 */
typedef struct
{
    FocMotorConfig_t config;
    FocMotorState_t state;

    FocPi_t d_axis_pi;
    FocPi_t q_axis_pi;

    FocCurrentReference_t current_reference;
    FocMotorFeedback_t feedback;

    AlphaBetaCurrent_t alpha_beta_current;
    DqCurrent_t dq_current;
    DqVoltage_t dq_voltage;

    AlphaBetaVoltage_t alpha_beta_voltage;
    PwmDuty_t pwm_duty;
} FocMotor_t;

/**
 * @brief Initializes one FOC motor control instance.
 * @note This function does not enable PWM or gate-driver hardware.
 */
void FocMotor_Init(FocMotor_t *motor, const FocMotorConfig_t *config);

/**
 * @brief Starts software current control from the stopped state.
 * @return True when the state changes to running.
 * @note Hardware safety checks must be completed by the caller.
 */
bool FocMotor_Start(FocMotor_t *motor);

/**
 * @brief Stops software current control and clears PI runtime state.
 */
void FocMotor_Stop(FocMotor_t *motor);

/**
 * @brief Enters the latched software fault state.
 */
void FocMotor_EnterFault(FocMotor_t *motor);

/**
 * @brief Updates the d-q current command.
 */
void FocMotor_SetCurrentReference(
    FocMotor_t *motor,
    FocCurrentReference_t current_reference);

/**
 * @brief Executes one complete FOC current-control iteration.
 * @param feedback Current measurements, electrical angle, and DC-link voltage.
 * @return Three normalized PWM duty commands.
 * @note Returns neutral PWM duty when the motor is not running.
 */
PwmDuty_t FocMotor_RunCurrentLoop(
    FocMotor_t *motor,
    const FocMotorFeedback_t *feedback);

#endif /* FOC_MOTOR_H_ */
