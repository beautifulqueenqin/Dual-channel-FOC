#include "app.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "bsp_current.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_safety.h"
#include "dual_motor.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/* Compile-time application parameters                                        */
/* -------------------------------------------------------------------------- */

#define APP_CURRENT_CALIBRATION_SAMPLES (256U)

/* Encoder continuity-test parameters. */
#define APP_ENCODER_TEST_MOTOR_ID      (MOTOR_ID_0)
#define APP_ENCODER_TEST_PERIOD_MS     (20U)
#define APP_ENCODER_RAW_RESOLUTION     (16384U)
#define APP_ENCODER_RAW_HALF_SCALE     (8192U)
#define APP_ENCODER_DIRECTION_DEADBAND (2)
#define APP_ENCODER_MAX_STEP_COUNTS    (4096)

/* -------------------------------------------------------------------------- */
/* Private types                                                              */
/* -------------------------------------------------------------------------- */

typedef enum
{
    APP_ENCODER_DIRECTION_UNKNOWN = 0,
    APP_ENCODER_DIRECTION_POSITIVE,
    APP_ENCODER_DIRECTION_NEGATIVE,
    APP_ENCODER_DIRECTION_STATIONARY
} AppEncoderDirection_t;

/* Runtime diagnostics intentionally remain visible in the debugger. */
typedef struct
{
    BspEncoderSample_t latest_sample;
    float electrical_angle_rad;

    uint16_t first_raw_angle;
    uint16_t last_raw_angle;
    int32_t last_step_counts;
    int32_t accumulated_counts;
    int32_t wrap_count;

    uint32_t sample_count;
    uint32_t invalid_sample_count;
    uint32_t large_step_count;

    AppEncoderDirection_t direction;
    bool initialized;
    bool sample_valid;
    bool continuity_ok;
} AppEncoderContinuity_t;

/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

static DualMotor_t app_dual_motor;
static bool app_dual_motor_initialized;
static bool app_current_calibrated;

/* Keep these arrays visible to the debugger during bring-up. */
static volatile BspEncoderSample_t
    app_encoder_sample[MOTOR_ID_COUNT];
static volatile bool
    app_encoder_sample_valid[MOTOR_ID_COUNT];
static volatile AppEncoderContinuity_t
    app_encoder_continuity[MOTOR_ID_COUNT];

/* -------------------------------------------------------------------------- */
/* Private function declarations                                              */
/* -------------------------------------------------------------------------- */

static void App_ProcessSafetyFault(void);
static bool App_CalibrateCurrentSensors(void);

static void App_TestEncoderRead(void);
static void App_TestEncoderContinuity(void);
static void App_InitializeEncoderDiagnostic(
    volatile AppEncoderContinuity_t *diagnostic,
    const BspEncoderSample_t *sample);
static bool App_UpdateElectricalAngle(
    MotorId_t motor_id,
    float mechanical_angle_rad,
    volatile AppEncoderContinuity_t *diagnostic);

static int32_t App_ComputeWrappedRawDelta(
    uint16_t previous_raw,
    uint16_t current_raw,
    int32_t *wrap_step);

/* -------------------------------------------------------------------------- */
/* Public application API                                                     */
/* -------------------------------------------------------------------------- */

void App_Init(void)
{
    BspSafety_Init();
    BspPwm_Init();

    BspEncoder_Init(AppConfig_GetEncoderConfig());
    App_TestEncoderRead();

    BspCurrent_Init(AppConfig_GetCurrentConfig());
    app_current_calibrated = App_CalibrateCurrentSensors();

    app_dual_motor_initialized =
        app_current_calibrated &&
        DualMotor_Init(
            &app_dual_motor,
            AppConfig_GetDualMotorConfig());
}

void App_Loop(void)
{
    /* Keep this loop non-blocking; hardware timers own fast control timing. */
    App_TestEncoderContinuity();
    App_ProcessSafetyFault();
}

/* -------------------------------------------------------------------------- */
/* Safety and calibration                                                     */
/* -------------------------------------------------------------------------- */

static void App_ProcessSafetyFault(void)
{
    /* Do not access the motor controller before initialization. */
    if (!app_dual_motor_initialized)
    {
        return;
    }

    /* Convert the BSP hardware fault into a dual-motor software fault. */
    if (BspSafety_IsFaultLatched())
    {
        DualMotor_EnterFault(&app_dual_motor);
    }
}

static bool App_CalibrateCurrentSensors(void)
{
    bool calibration_ok;

    /* Never calibrate when a hardware fault is active. */
    if (BspSafety_IsFaultLatched())
    {
        return false;
    }

    /* Arm ADC injected conversions before starting the PWM trigger. */
    if (!BspCurrent_Start())
    {
        return false;
    }

    /* EN_GATE is still low, so PWM can safely provide ADC trigger events. */
    if (!BspPwm_Start())
    {
        BspCurrent_Stop();
        return false;
    }

    calibration_ok =
        BspCurrent_CalibrateOffsets(
            APP_CURRENT_CALIBRATION_SAMPLES);

    /* Always stop PWM and ADC after calibration. */
    BspPwm_Stop();
    BspCurrent_Stop();

    return calibration_ok;
}

/* -------------------------------------------------------------------------- */
/* Encoder bring-up diagnostics                                                */
/* -------------------------------------------------------------------------- */

static void App_TestEncoderRead(void)
{
    MotorId_t motor_id;

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        BspEncoderSample_t sample;
        volatile AppEncoderContinuity_t *diagnostic;
        bool read_ok;

        diagnostic = &app_encoder_continuity[motor_id];
        read_ok = BspEncoder_ReadSample(motor_id, &sample);

        if (!read_ok)
        {
            diagnostic->sample_valid = false;
            diagnostic->continuity_ok = false;
            app_encoder_sample_valid[motor_id] = false;
            continue;
        }

        app_encoder_sample[motor_id] = sample;
        app_encoder_sample_valid[motor_id] = true;

        App_InitializeEncoderDiagnostic(diagnostic, &sample);

        if (!App_UpdateElectricalAngle(
                motor_id,
                sample.mechanical_angle_rad,
                diagnostic))
        {
            diagnostic->continuity_ok = false;
        }
    }
}

static void App_TestEncoderContinuity(void)
{
    static uint32_t previous_test_tick;
    static bool test_scheduler_initialized;

    const MotorId_t motor_id = APP_ENCODER_TEST_MOTOR_ID;
    volatile AppEncoderContinuity_t *diagnostic;
    BspEncoderSample_t sample;
    uint32_t current_tick;
    uint32_t elapsed_ms;
    int32_t wrap_step;
    int32_t delta_counts;

    current_tick = HAL_GetTick();

    if (!test_scheduler_initialized)
    {
        test_scheduler_initialized = true;
        previous_test_tick = current_tick;
    }

    elapsed_ms = current_tick - previous_test_tick;

    if (elapsed_ms < APP_ENCODER_TEST_PERIOD_MS)
    {
        return;
    }

    previous_test_tick = current_tick;
    diagnostic = &app_encoder_continuity[motor_id];

    if (!BspEncoder_ReadSample(motor_id, &sample))
    {
        diagnostic->sample_valid = false;
        diagnostic->invalid_sample_count++;
        app_encoder_sample_valid[motor_id] = false;
        return;
    }

    app_encoder_sample[motor_id] = sample;
    app_encoder_sample_valid[motor_id] = true;
    diagnostic->latest_sample = sample;
    diagnostic->sample_valid = true;
    diagnostic->sample_count++;

    if (!diagnostic->initialized)
    {
        App_InitializeEncoderDiagnostic(diagnostic, &sample);

        if (!App_UpdateElectricalAngle(
                motor_id,
                sample.mechanical_angle_rad,
                diagnostic))
        {
            diagnostic->continuity_ok = false;
        }

        return;
    }

    if (!App_UpdateElectricalAngle(
            motor_id,
            sample.mechanical_angle_rad,
            diagnostic))
    {
        diagnostic->continuity_ok = false;
    }

    delta_counts =
        App_ComputeWrappedRawDelta(
            diagnostic->last_raw_angle,
            sample.raw_angle,
            &wrap_step);

    diagnostic->last_step_counts = delta_counts;
    diagnostic->accumulated_counts += delta_counts;
    diagnostic->wrap_count += wrap_step;
    diagnostic->last_raw_angle = sample.raw_angle;

    if (delta_counts > APP_ENCODER_DIRECTION_DEADBAND)
    {
        diagnostic->direction = APP_ENCODER_DIRECTION_POSITIVE;
    }
    else if (delta_counts < -APP_ENCODER_DIRECTION_DEADBAND)
    {
        diagnostic->direction = APP_ENCODER_DIRECTION_NEGATIVE;
    }
    else
    {
        diagnostic->direction = APP_ENCODER_DIRECTION_STATIONARY;
    }

    /* A large corrected step usually means a missed sample or wiring noise. */
    if ((delta_counts > APP_ENCODER_MAX_STEP_COUNTS) ||
        (delta_counts < -APP_ENCODER_MAX_STEP_COUNTS))
    {
        diagnostic->large_step_count++;
        diagnostic->continuity_ok = false;
    }
}

static void App_InitializeEncoderDiagnostic(
    volatile AppEncoderContinuity_t *diagnostic,
    const BspEncoderSample_t *sample)
{
    if ((diagnostic == NULL) || (sample == NULL))
    {
        return;
    }

    diagnostic->latest_sample = *sample;
    diagnostic->electrical_angle_rad = 0.0f;
    diagnostic->first_raw_angle = sample->raw_angle;
    diagnostic->last_raw_angle = sample->raw_angle;
    diagnostic->last_step_counts = 0;
    diagnostic->accumulated_counts = 0;
    diagnostic->wrap_count = 0;
    diagnostic->sample_count = 1U;
    diagnostic->invalid_sample_count = 0U;
    diagnostic->large_step_count = 0U;
    diagnostic->direction = APP_ENCODER_DIRECTION_UNKNOWN;
    diagnostic->initialized = true;
    diagnostic->sample_valid = true;
    diagnostic->continuity_ok = true;
}

static bool App_UpdateElectricalAngle(
    MotorId_t motor_id,
    float mechanical_angle_rad,
    volatile AppEncoderContinuity_t *diagnostic)
{
    const BspEncoderConfig_t *encoder_config;
    float electrical_angle_rad;

    if ((diagnostic == NULL) ||
        ((unsigned int)motor_id >= (unsigned int)MOTOR_ID_COUNT))
    {
        return false;
    }

    encoder_config = AppConfig_GetEncoderConfig();

    if (encoder_config == NULL)
    {
        return false;
    }

    if (!BspEncoder_ComputeElectricalAngle(
            mechanical_angle_rad,
            &encoder_config->motor_config[motor_id],
            &electrical_angle_rad))
    {
        return false;
    }

    diagnostic->electrical_angle_rad = electrical_angle_rad;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Encoder math helpers                                                       */
/* -------------------------------------------------------------------------- */

static int32_t App_ComputeWrappedRawDelta(
    uint16_t previous_raw,
    uint16_t current_raw,
    int32_t *wrap_step)
{
    int32_t delta_counts;

    if (wrap_step == NULL)
    {
        return 0;
    }

    delta_counts =
        (int32_t)current_raw - (int32_t)previous_raw;
    *wrap_step = 0;

    /* Convert the 0/16383 discontinuity into a small signed step. */
    if (delta_counts > APP_ENCODER_RAW_HALF_SCALE)
    {
        delta_counts -= APP_ENCODER_RAW_RESOLUTION;
        *wrap_step = -1;
    }
    else if (delta_counts < -APP_ENCODER_RAW_HALF_SCALE)
    {
        delta_counts += APP_ENCODER_RAW_RESOLUTION;
        *wrap_step = 1;
    }

    return delta_counts;
}
