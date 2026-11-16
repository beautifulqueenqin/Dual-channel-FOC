/* Application-specific motor-control configuration. */
#include "app_config.h"

#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* Application timing                                                         */
/* -------------------------------------------------------------------------- */

#define APP_CURRENT_LOOP_PERIOD_S (0.0001f)

/* -------------------------------------------------------------------------- */
/* Safe bring-up defaults                                                     */
/* -------------------------------------------------------------------------- */

/* Keep commands disabled until the real hardware parameters are confirmed. */
#define APP_SAFE_CURRENT_LIMIT_A      (0.0f)
#define APP_SAFE_VOLTAGE_LIMIT_V      (0.0f)
#define APP_SAFE_AMPERE_PER_COUNT_A   (0.0f)
#define APP_SAFE_OFFSET_COUNT         (0.0f)

/* -------------------------------------------------------------------------- */
/* Encoder defaults                                                           */
/* -------------------------------------------------------------------------- */

/* Temporary value: verify the exact DJI 2312 motor variant. */
#define APP_ENCODER_POLE_PAIRS        (4U)
#define APP_ENCODER_OFFSET_RAD        (0.0f)
#define APP_ENCODER_INVERT_DIRECTION  (false)

/* -------------------------------------------------------------------------- */
/* Dual-motor FOC configuration                                               */
/* -------------------------------------------------------------------------- */

static const DualMotorConfig_t app_dual_motor_config =
{
    .motor_config =
    {
        [MOTOR_ID_0] =
        {
            .motor_id = MOTOR_ID_0,
            .current_loop_period_s = APP_CURRENT_LOOP_PERIOD_S,
            .d_axis_current_limit_A = APP_SAFE_CURRENT_LIMIT_A,
            .q_axis_current_limit_A = APP_SAFE_CURRENT_LIMIT_A,
            .d_axis_pi =
            {
                .kp = 0.0f,
                .ki = 0.0f,
                .output_min = -APP_SAFE_VOLTAGE_LIMIT_V,
                .output_max = APP_SAFE_VOLTAGE_LIMIT_V
            },
            .q_axis_pi =
            {
                .kp = 0.0f,
                .ki = 0.0f,
                .output_min = -APP_SAFE_VOLTAGE_LIMIT_V,
                .output_max = APP_SAFE_VOLTAGE_LIMIT_V
            }
        },
        [MOTOR_ID_1] =
        {
            .motor_id = MOTOR_ID_1,
            .current_loop_period_s = APP_CURRENT_LOOP_PERIOD_S,
            .d_axis_current_limit_A = APP_SAFE_CURRENT_LIMIT_A,
            .q_axis_current_limit_A = APP_SAFE_CURRENT_LIMIT_A,
            .d_axis_pi =
            {
                .kp = 0.0f,
                .ki = 0.0f,
                .output_min = -APP_SAFE_VOLTAGE_LIMIT_V,
                .output_max = APP_SAFE_VOLTAGE_LIMIT_V
            },
            .q_axis_pi =
            {
                .kp = 0.0f,
                .ki = 0.0f,
                .output_min = -APP_SAFE_VOLTAGE_LIMIT_V,
                .output_max = APP_SAFE_VOLTAGE_LIMIT_V
            }
        }
    }
};

/* -------------------------------------------------------------------------- */
/* Current-sensing configuration                                              */
/* -------------------------------------------------------------------------- */

static const BspCurrentConfig_t app_current_config =
{
    .motor_config =
    {
        [MOTOR_ID_0] =
        {
            .ampere_per_count = APP_SAFE_AMPERE_PER_COUNT_A,
            .offset_a_count = APP_SAFE_OFFSET_COUNT,
            .offset_b_count = APP_SAFE_OFFSET_COUNT
        },
        [MOTOR_ID_1] =
        {
            .ampere_per_count = APP_SAFE_AMPERE_PER_COUNT_A,
            .offset_a_count = APP_SAFE_OFFSET_COUNT,
            .offset_b_count = APP_SAFE_OFFSET_COUNT
        }
    }
};

/* -------------------------------------------------------------------------- */
/* Encoder configuration                                                      */
/* -------------------------------------------------------------------------- */

static const BspEncoderConfig_t app_encoder_config =
{
    .motor_config =
    {
        [MOTOR_ID_0] =
        {
            .pole_pairs = APP_ENCODER_POLE_PAIRS,
            .electrical_offset_rad = APP_ENCODER_OFFSET_RAD,
            .invert_direction = APP_ENCODER_INVERT_DIRECTION
        },
        [MOTOR_ID_1] =
        {
            .pole_pairs = APP_ENCODER_POLE_PAIRS,
            .electrical_offset_rad = APP_ENCODER_OFFSET_RAD,
            .invert_direction = APP_ENCODER_INVERT_DIRECTION
        }
    }
};

/* -------------------------------------------------------------------------- */
/* Configuration accessors                                                    */
/* -------------------------------------------------------------------------- */

const DualMotorConfig_t *AppConfig_GetDualMotorConfig(void)
{
    return &app_dual_motor_config;
}

const BspCurrentConfig_t *AppConfig_GetCurrentConfig(void)
{
    return &app_current_config;
}

const BspEncoderConfig_t *AppConfig_GetEncoderConfig(void)
{
    return &app_encoder_config;
}
