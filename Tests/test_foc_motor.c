#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "foc_motor.h"

#define TEST_TOLERANCE (1.0e-5f)

/* Shared configuration used by all motor-control tests. */
static const FocMotorConfig_t test_motor_config =
{
    .motor_id = MOTOR_ID_0,
    .current_loop_period_s = 0.001f,
    .d_axis_current_limit_A = 2.0f,
    .q_axis_current_limit_A = 5.0f,

    .d_axis_pi =
    {
        .kp = 2.0f,
        .ki = 10.0f,
        .output_min = -6.0f,
        .output_max = 6.0f
    },

    .q_axis_pi =
    {
        .kp = 2.0f,
        .ki = 10.0f,
        .output_min = -6.0f,
        .output_max = 6.0f
    }
};

/**
 * @brief Checks whether a floating-point result is within tolerance.
 *
 * @return 0 when the check passes, otherwise 1.
 */
static int Test_CheckNear(
    const char *name,
    float actual,
    float expected,
    float tolerance)
{
    const float difference = fabsf(actual - expected);

    if ((!isfinite(actual)) || (difference > tolerance))
    {
        printf(
            "[FAIL] %s: expected %.6f, actual %.6f\n",
            name,
            expected,
            actual);

        return 1;
    }

    printf(
        "[PASS] %s: %.6f\n",
        name,
        actual);

    return 0;
}

/**
 * @brief Checks whether two integer values are equal.
 *
 * @return 0 when the check passes, otherwise 1.
 */
static int Test_CheckEqualInt(
    const char *name,
    int actual,
    int expected)
{
    if (actual != expected)
    {
        printf(
            "[FAIL] %s: expected %d, actual %d\n",
            name,
            expected,
            actual);

        return 1;
    }

    printf(
        "[PASS] %s: %d\n",
        name,
        actual);

    return 0;
}

/**
 * @brief Verifies the initial state of one motor-control instance.
 */
static int Test_MotorInitialization(void)
{
    FocMotor_t motor;
    int failure_count = 0;

    printf("[TEST] Motor initialization\n");

    FocMotor_Init(&motor, &test_motor_config);

    failure_count += Test_CheckEqualInt(
        "Initial state is stopped",
        (int)motor.state,
        (int)FOC_MOTOR_STATE_STOPPED);

    failure_count += Test_CheckNear(
        "Initial d-axis reference",
        motor.current_reference.d_A,
        0.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Initial q-axis reference",
        motor.current_reference.q_A,
        0.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Initial PWM phase A",
        motor.pwm_duty.phase_a,
        0.5f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Initial PWM phase B",
        motor.pwm_duty.phase_b,
        0.5f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Initial PWM phase C",
        motor.pwm_duty.phase_c,
        0.5f,
        TEST_TOLERANCE);

    return failure_count;
}

/**
 * @brief Verifies motor start behavior and current-reference limiting.
 */
static int Test_MotorStartAndReferenceLimiting(void)
{
    FocMotor_t motor;

    const FocCurrentReference_t requested_reference =
    {
        .d_A = 3.0f,
        .q_A = -7.0f
    };

    bool start_result;
    bool second_start_result;
    int failure_count = 0;

    printf("[TEST] Motor start and reference limiting\n");

    FocMotor_Init(&motor, &test_motor_config);

    /* Current commands must be ignored while the motor is stopped. */
    FocMotor_SetCurrentReference(
        &motor,
        requested_reference);

    failure_count += Test_CheckNear(
        "Stopped motor rejects d-axis reference",
        motor.current_reference.d_A,
        0.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Stopped motor rejects q-axis reference",
        motor.current_reference.q_A,
        0.0f,
        TEST_TOLERANCE);

    start_result = FocMotor_Start(&motor);

    failure_count += Test_CheckEqualInt(
        "First start succeeds",
        (int)start_result,
        (int)true);

    failure_count += Test_CheckEqualInt(
        "State changes to running",
        (int)motor.state,
        (int)FOC_MOTOR_STATE_RUNNING);

    FocMotor_SetCurrentReference(
        &motor,
        requested_reference);

    failure_count += Test_CheckNear(
        "d-axis reference is limited",
        motor.current_reference.d_A,
        2.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "q-axis reference is limited",
        motor.current_reference.q_A,
        -5.0f,
        TEST_TOLERANCE);

    second_start_result = FocMotor_Start(&motor);

    failure_count += Test_CheckEqualInt(
        "Second start is rejected",
        (int)second_start_result,
        (int)false);

    return failure_count;
}

/**
 * @brief Verifies one complete FOC current-control iteration.
 */
static int Test_FullCurrentLoop(void)
{
    FocMotor_t motor;

    const FocCurrentReference_t current_reference =
    {
        .d_A = 0.0f,
        .q_A = 1.0f
    };

    const FocMotorFeedback_t feedback =
    {
        .phase_current =
        {
            .phase_a_A = 1.0f,
            .phase_b_A = -0.5f,
            .phase_c_A = -0.5f
        },
        .electrical_angle_rad = 0.0f,
        .dc_bus_voltage_V = 12.0f
    };

    PwmDuty_t pwm_duty;
    int failure_count = 0;

    printf("[TEST] Full FOC current loop\n");

    FocMotor_Init(&motor, &test_motor_config);

    failure_count += Test_CheckEqualInt(
        "Current-loop motor starts",
        (int)FocMotor_Start(&motor),
        (int)true);

    FocMotor_SetCurrentReference(
        &motor,
        current_reference);

    pwm_duty = FocMotor_RunCurrentLoop(
        &motor,
        &feedback);

    /* Clarke transform: abc current to stationary alpha-beta current. */
    failure_count += Test_CheckNear(
        "Clarke alpha current",
        motor.alpha_beta_current.alpha_A,
        1.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Clarke beta current",
        motor.alpha_beta_current.beta_A,
        0.0f,
        TEST_TOLERANCE);

    /* Park transform: stationary alpha-beta current to rotating d-q current. */
    failure_count += Test_CheckNear(
        "Park d-axis current",
        motor.dq_current.d_A,
        1.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Park q-axis current",
        motor.dq_current.q_A,
        0.0f,
        TEST_TOLERANCE);

    /* First PI step with zero initial integrator state. */
    failure_count += Test_CheckNear(
        "d-axis PI integrator",
        motor.d_axis_pi.integrator,
        -0.01f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "q-axis PI integrator",
        motor.q_axis_pi.integrator,
        0.01f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "d-axis voltage command",
        motor.dq_voltage.d_V,
        -2.01f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "q-axis voltage command",
        motor.dq_voltage.q_V,
        2.01f,
        TEST_TOLERANCE);

    /* Inverse Park transform: rotating d-q voltage to alpha-beta voltage. */
    failure_count += Test_CheckNear(
        "Inverse Park alpha voltage",
        motor.alpha_beta_voltage.alpha_V,
        -2.01f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Inverse Park beta voltage",
        motor.alpha_beta_voltage.beta_V,
        2.01f,
        TEST_TOLERANCE);

    /* Min-max SVPWM with a 12 V DC bus. */
    failure_count += Test_CheckNear(
        "SVPWM phase A duty",
        pwm_duty.phase_a,
        0.301845f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "SVPWM phase B duty",
        pwm_duty.phase_b,
        0.698155f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "SVPWM phase C duty",
        pwm_duty.phase_c,
        0.408036f,
        TEST_TOLERANCE);

    return failure_count;
}

int main(void)
{
    int failure_count = 0;

    failure_count += Test_MotorInitialization();
    failure_count += Test_MotorStartAndReferenceLimiting();
    failure_count += Test_FullCurrentLoop();

    if (failure_count != 0)
    {
        printf("[RESULT] %d check(s) failed.\n", failure_count);
        return 1;
    }

    printf("[RESULT] All checks passed.\n");
    return 0;
}
