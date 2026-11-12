#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "dual_motor.h"

#define TEST_TOLERANCE (1.0e-5f)

/* Shared configuration used by all dual-motor controller tests. */
static const DualMotorConfig_t test_dual_motor_config =
{
    .motor_config =
    {
        [MOTOR_ID_0] =
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
        },
        [MOTOR_ID_1] =
        {
            .motor_id = MOTOR_ID_1,
            .current_loop_period_s = 0.001f,
            .d_axis_current_limit_A = 1.0f,
            .q_axis_current_limit_A = 3.0f,
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
        }
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
 * @brief Verifies dual-motor initialization and configuration validation.
 */
static int Test_DualMotorInitialization(void)
{
    DualMotor_t dual_motor;
    DualMotorConfig_t invalid_config = test_dual_motor_config;
    int failure_count = 0;

    printf("[TEST] Dual-motor initialization\n");

    failure_count += Test_CheckEqualInt(
        "Valid dual configuration is accepted",
        (int)DualMotor_Init(&dual_motor, &test_dual_motor_config),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Null controller initialization is rejected",
        (int)DualMotor_Init(NULL, &test_dual_motor_config),
        (int)false);

    failure_count += Test_CheckEqualInt(
        "Shared fault starts cleared",
        (int)DualMotor_IsFaultLatched(&dual_motor),
        (int)false);

    failure_count += Test_CheckEqualInt(
        "Null controller is treated as faulted",
        (int)DualMotor_IsFaultLatched(NULL),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 0 starts stopped",
        (int)dual_motor.motor[MOTOR_ID_0].state,
        (int)FOC_MOTOR_STATE_STOPPED);

    failure_count += Test_CheckEqualInt(
        "Motor 1 starts stopped",
        (int)dual_motor.motor[MOTOR_ID_1].state,
        (int)FOC_MOTOR_STATE_STOPPED);

    failure_count += Test_CheckEqualInt(
        "Motor 0 starts before rejected initialization",
        (int)DualMotor_StartMotor(&dual_motor, MOTOR_ID_0),
        (int)true);

    invalid_config.motor_config[MOTOR_ID_1].motor_id = MOTOR_ID_0;

    failure_count += Test_CheckEqualInt(
        "Mismatched configuration is rejected",
        (int)DualMotor_Init(&dual_motor, &invalid_config),
        (int)false);

    failure_count += Test_CheckEqualInt(
        "Rejected initialization preserves motor 0 state",
        (int)dual_motor.motor[MOTOR_ID_0].state,
        (int)FOC_MOTOR_STATE_RUNNING);

    return failure_count;
}

/**
 * @brief Verifies independent motor starts and current-reference limits.
 */
static int Test_DualMotorStartAndReferenceLimiting(void)
{
    DualMotor_t dual_motor;

    const FocCurrentReference_t motor_0_reference =
    {
        .d_A = 3.0f,
        .q_A = -7.0f
    };

    const FocCurrentReference_t motor_1_reference =
    {
        .d_A = 2.0f,
        .q_A = -4.0f
    };

    const MotorId_t invalid_motor_id = (MotorId_t)-1;
    int failure_count = 0;

    printf("[TEST] Dual-motor start and reference limiting\n");

    failure_count += Test_CheckEqualInt(
        "Dual controller initializes",
        (int)DualMotor_Init(&dual_motor, &test_dual_motor_config),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Stopped motor rejects reference",
        (int)DualMotor_SetCurrentReference(
            &dual_motor,
            MOTOR_ID_0,
            motor_0_reference),
        (int)false);

    failure_count += Test_CheckEqualInt(
        "Invalid motor ID is rejected",
        (int)DualMotor_StartMotor(
            &dual_motor,
            invalid_motor_id),
        (int)false);

    failure_count += Test_CheckEqualInt(
        "Motor 0 starts",
        (int)DualMotor_StartMotor(&dual_motor, MOTOR_ID_0),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 0 accepts reference",
        (int)DualMotor_SetCurrentReference(
            &dual_motor,
            MOTOR_ID_0,
            motor_0_reference),
        (int)true);

    failure_count += Test_CheckNear(
        "Motor 0 d-axis reference is limited",
        dual_motor.motor[MOTOR_ID_0].current_reference.d_A,
        2.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 0 q-axis reference is limited",
        dual_motor.motor[MOTOR_ID_0].current_reference.q_A,
        -5.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckEqualInt(
        "Motor 1 remains stopped",
        (int)dual_motor.motor[MOTOR_ID_1].state,
        (int)FOC_MOTOR_STATE_STOPPED);

    failure_count += Test_CheckEqualInt(
        "Motor 1 starts independently",
        (int)DualMotor_StartMotor(&dual_motor, MOTOR_ID_1),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 1 accepts reference",
        (int)DualMotor_SetCurrentReference(
            &dual_motor,
            MOTOR_ID_1,
            motor_1_reference),
        (int)true);

    failure_count += Test_CheckNear(
        "Motor 1 d-axis reference is limited",
        dual_motor.motor[MOTOR_ID_1].current_reference.d_A,
        1.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 1 q-axis reference is limited",
        dual_motor.motor[MOTOR_ID_1].current_reference.q_A,
        -3.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckEqualInt(
        "Motor 0 stops independently",
        (int)DualMotor_StopMotor(&dual_motor, MOTOR_ID_0),
        (int)true);

    failure_count += Test_CheckNear(
        "Stopped motor 0 clears q-axis reference",
        dual_motor.motor[MOTOR_ID_0].current_reference.q_A,
        0.0f,
        TEST_TOLERANCE);

    DualMotor_StopAll(&dual_motor);

    failure_count += Test_CheckEqualInt(
        "Stop all stops motor 0",
        (int)dual_motor.motor[MOTOR_ID_0].state,
        (int)FOC_MOTOR_STATE_STOPPED);

    failure_count += Test_CheckEqualInt(
        "Stop all stops motor 1",
        (int)dual_motor.motor[MOTOR_ID_1].state,
        (int)FOC_MOTOR_STATE_STOPPED);

    return failure_count;
}

/**
 * @brief Verifies one independent current-control iteration per motor.
 */
static int Test_DualMotorCurrentLoops(void)
{
    DualMotor_t dual_motor;

    const FocCurrentReference_t motor_0_reference =
    {
        .d_A = 0.0f,
        .q_A = 1.0f
    };

    const FocCurrentReference_t motor_1_reference =
    {
        .d_A = 0.0f,
        .q_A = -1.0f
    };

    const DualMotorFeedback_t feedback =
    {
        .motor_feedback =
        {
            [MOTOR_ID_0] =
            {
                .phase_current =
                {
                    .phase_a_A = 1.0f,
                    .phase_b_A = -0.5f,
                    .phase_c_A = -0.5f
                },
                .electrical_angle_rad = 0.0f,
                .dc_bus_voltage_V = 12.0f
            },
            [MOTOR_ID_1] =
            {
                .phase_current =
                {
                    .phase_a_A = -1.0f,
                    .phase_b_A = 0.5f,
                    .phase_c_A = 0.5f
                },
                .electrical_angle_rad = 0.0f,
                .dc_bus_voltage_V = 12.0f
            }
        }
    };

    DualMotorPwmOutput_t pwm_output;
    int failure_count = 0;

    printf("[TEST] Independent dual-motor current loops\n");

    failure_count += Test_CheckEqualInt(
        "Dual controller initializes for current loops",
        (int)DualMotor_Init(&dual_motor, &test_dual_motor_config),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 0 starts for current loop",
        (int)DualMotor_StartMotor(&dual_motor, MOTOR_ID_0),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 1 starts for current loop",
        (int)DualMotor_StartMotor(&dual_motor, MOTOR_ID_1),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 0 current reference is accepted",
        (int)DualMotor_SetCurrentReference(
            &dual_motor,
            MOTOR_ID_0,
            motor_0_reference),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 1 current reference is accepted",
        (int)DualMotor_SetCurrentReference(
            &dual_motor,
            MOTOR_ID_1,
            motor_1_reference),
        (int)true);

    pwm_output = DualMotor_RunCurrentLoop(
        &dual_motor,
        &feedback);

    failure_count += Test_CheckNear(
        "Motor 0 Clarke alpha current",
        dual_motor.motor[MOTOR_ID_0].alpha_beta_current.alpha_A,
        1.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 0 d-axis voltage",
        dual_motor.motor[MOTOR_ID_0].dq_voltage.d_V,
        -2.01f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 0 q-axis voltage",
        dual_motor.motor[MOTOR_ID_0].dq_voltage.q_V,
        2.01f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 0 PWM phase A",
        pwm_output.pwm_duty[MOTOR_ID_0].phase_a,
        0.301845f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 0 PWM phase B",
        pwm_output.pwm_duty[MOTOR_ID_0].phase_b,
        0.698155f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 0 PWM phase C",
        pwm_output.pwm_duty[MOTOR_ID_0].phase_c,
        0.408036f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 1 Clarke alpha current",
        dual_motor.motor[MOTOR_ID_1].alpha_beta_current.alpha_A,
        -1.0f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 1 d-axis voltage",
        dual_motor.motor[MOTOR_ID_1].dq_voltage.d_V,
        2.01f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 1 q-axis voltage",
        dual_motor.motor[MOTOR_ID_1].dq_voltage.q_V,
        -2.01f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 1 PWM phase A",
        pwm_output.pwm_duty[MOTOR_ID_1].phase_a,
        0.698155f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 1 PWM phase B",
        pwm_output.pwm_duty[MOTOR_ID_1].phase_b,
        0.301845f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Motor 1 PWM phase C",
        pwm_output.pwm_duty[MOTOR_ID_1].phase_c,
        0.591964f,
        TEST_TOLERANCE);

    return failure_count;
}

/**
 * @brief Verifies shared fault latching, neutral output, and reset behavior.
 */
static int Test_DualMotorFaultHandling(void)
{
    DualMotor_t dual_motor;
    const DualMotorFeedback_t feedback = {0};
    DualMotorPwmOutput_t pwm_output;
    int failure_count = 0;

    printf("[TEST] Shared dual-motor fault handling\n");

    failure_count += Test_CheckEqualInt(
        "Dual controller initializes for fault test",
        (int)DualMotor_Init(&dual_motor, &test_dual_motor_config),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 0 starts before fault",
        (int)DualMotor_StartMotor(&dual_motor, MOTOR_ID_0),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 1 starts before fault",
        (int)DualMotor_StartMotor(&dual_motor, MOTOR_ID_1),
        (int)true);

    DualMotor_EnterFault(&dual_motor);

    failure_count += Test_CheckEqualInt(
        "Shared fault is latched",
        (int)DualMotor_IsFaultLatched(&dual_motor),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Motor 0 enters fault",
        (int)dual_motor.motor[MOTOR_ID_0].state,
        (int)FOC_MOTOR_STATE_FAULT);

    failure_count += Test_CheckEqualInt(
        "Motor 1 enters fault",
        (int)dual_motor.motor[MOTOR_ID_1].state,
        (int)FOC_MOTOR_STATE_FAULT);

    failure_count += Test_CheckEqualInt(
        "Faulted controller rejects a start",
        (int)DualMotor_StartMotor(&dual_motor, MOTOR_ID_0),
        (int)false);

    failure_count += Test_CheckEqualInt(
        "Faulted controller rejects a current reference",
        (int)DualMotor_SetCurrentReference(
            &dual_motor,
            MOTOR_ID_0,
            (FocCurrentReference_t){0.0f, 1.0f}),
        (int)false);

    DualMotor_StopAll(&dual_motor);

    failure_count += Test_CheckEqualInt(
        "Stop all preserves shared fault",
        (int)DualMotor_IsFaultLatched(&dual_motor),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Stop all preserves motor 0 fault",
        (int)dual_motor.motor[MOTOR_ID_0].state,
        (int)FOC_MOTOR_STATE_FAULT);

    failure_count += Test_CheckEqualInt(
        "Stop all preserves motor 1 fault",
        (int)dual_motor.motor[MOTOR_ID_1].state,
        (int)FOC_MOTOR_STATE_FAULT);

    pwm_output = DualMotor_RunCurrentLoop(
        &dual_motor,
        &feedback);

    failure_count += Test_CheckNear(
        "Faulted motor 0 PWM phase A is neutral",
        pwm_output.pwm_duty[MOTOR_ID_0].phase_a,
        0.5f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Faulted motor 0 PWM phase B is neutral",
        pwm_output.pwm_duty[MOTOR_ID_0].phase_b,
        0.5f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Faulted motor 0 PWM phase C is neutral",
        pwm_output.pwm_duty[MOTOR_ID_0].phase_c,
        0.5f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Faulted motor 1 PWM phase A is neutral",
        pwm_output.pwm_duty[MOTOR_ID_1].phase_a,
        0.5f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Faulted motor 1 PWM phase B is neutral",
        pwm_output.pwm_duty[MOTOR_ID_1].phase_b,
        0.5f,
        TEST_TOLERANCE);

    failure_count += Test_CheckNear(
        "Faulted motor 1 PWM phase C is neutral",
        pwm_output.pwm_duty[MOTOR_ID_1].phase_c,
        0.5f,
        TEST_TOLERANCE);

    failure_count += Test_CheckEqualInt(
        "Fault clear succeeds once",
        (int)DualMotor_ClearFault(&dual_motor),
        (int)true);

    failure_count += Test_CheckEqualInt(
        "Shared fault is cleared",
        (int)DualMotor_IsFaultLatched(&dual_motor),
        (int)false);

    failure_count += Test_CheckEqualInt(
        "Motor 0 returns to stopped",
        (int)dual_motor.motor[MOTOR_ID_0].state,
        (int)FOC_MOTOR_STATE_STOPPED);

    failure_count += Test_CheckEqualInt(
        "Motor 1 returns to stopped",
        (int)dual_motor.motor[MOTOR_ID_1].state,
        (int)FOC_MOTOR_STATE_STOPPED);

    failure_count += Test_CheckEqualInt(
        "Second fault clear is rejected",
        (int)DualMotor_ClearFault(&dual_motor),
        (int)false);

    return failure_count;
}

int main(void)
{
    int failure_count = 0;

    failure_count += Test_DualMotorInitialization();
    failure_count += Test_DualMotorStartAndReferenceLimiting();
    failure_count += Test_DualMotorCurrentLoops();
    failure_count += Test_DualMotorFaultHandling();

    if (failure_count != 0)
    {
        printf("[RESULT] %d check(s) failed.\n", failure_count);
        return 1;
    }

    printf("[RESULT] All checks passed.\n");
    return 0;
}
