/* ADC current-sampling hardware adaptation. */
#include "bsp_current.h"

#include <stddef.h>
#include <stdint.h>

#include "adc.h"

typedef struct
{
    BspCurrentMotorConfig_t config;

    volatile uint16_t raw_a_count;
    volatile uint16_t raw_b_count;
    volatile uint32_t sample_sequence;
} BspCurrentMotorState_t;

#define BSP_CURRENT_CALIBRATION_TIMEOUT_MS  (1000U)

static bool BspCurrent_TryReadRawSample(
    const BspCurrentMotorState_t *state,
    uint32_t *last_sequence,
    uint16_t *raw_a_count,
    uint16_t *raw_b_count);

static BspCurrentMotorState_t
    bsp_current_motor[MOTOR_ID_COUNT];

static bool BspCurrent_IsValidMotorId(
    MotorId_t motor_id);

void BspCurrent_Init(
    const BspCurrentConfig_t *config)
{
    MotorId_t motor_id;

    BspCurrent_Stop();

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        if (config != NULL)
        {
            bsp_current_motor[motor_id].config =
                config->motor_config[motor_id];
        }
        else
        {
            bsp_current_motor[motor_id].config =
                (BspCurrentMotorConfig_t){0};
        }

        bsp_current_motor[motor_id].raw_a_count = 0U;
        bsp_current_motor[motor_id].raw_b_count = 0U;
        bsp_current_motor[motor_id].sample_sequence = 0U;
    }
}

bool BspCurrent_Start(void)
{
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        BspCurrent_Stop();
        return false;
    }

    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK)
    {
        BspCurrent_Stop();
        return false;
    }

    return true;
}

void BspCurrent_Stop(void)
{
    MotorId_t motor_id;

    (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
    (void)HAL_ADCEx_InjectedStop_IT(&hadc2);

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        bsp_current_motor[motor_id].sample_sequence = 0U;
    }
}

bool BspCurrent_GetPhaseCurrent(
    MotorId_t motor_id,
    PhaseCurrent_t *phase_current)
{
    const BspCurrentMotorState_t *state;
    BspCurrentMotorConfig_t config;
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint16_t raw_a_count;
    uint16_t raw_b_count;
    float phase_a_A;
    float phase_b_A;

    if ((!BspCurrent_IsValidMotorId(motor_id)) ||
        (phase_current == NULL))
    {
        return false;
    }

    state = &bsp_current_motor[motor_id];

    sequence_before = state->sample_sequence;

    if (sequence_before == 0U)
    {
        return false;
    }

    raw_a_count = state->raw_a_count;
    raw_b_count = state->raw_b_count;
    config = state->config;

    sequence_after = state->sample_sequence;

    /*
     * The ISR updated the sample during the read.
     */
    if (sequence_before != sequence_after)
    {
        return false;
    }

    phase_a_A =
        ((float)raw_a_count - config.offset_a_count) *
        config.ampere_per_count;

    phase_b_A =
        ((float)raw_b_count - config.offset_b_count) *
        config.ampere_per_count;

    phase_current->phase_a_A = phase_a_A;
    phase_current->phase_b_A = phase_b_A;

    /*
     * Reconstruct the third phase from Kirchhoff's current law.
     */
    phase_current->phase_c_A =
        -(phase_a_A + phase_b_A);

    return true;
}

bool BspCurrent_IsSampleReady(
    MotorId_t motor_id)
{
    if (!BspCurrent_IsValidMotorId(motor_id))
    {
        return false;
    }

    return (bsp_current_motor[motor_id].sample_sequence != 0U);
}

bool BspCurrent_CalibrateOffsets(
    uint32_t sample_count)
{
    uint64_t sum_a[MOTOR_ID_COUNT] = {0U};
    uint64_t sum_b[MOTOR_ID_COUNT] = {0U};

    uint32_t last_sequence[MOTOR_ID_COUNT];
    uint32_t collected_samples[MOTOR_ID_COUNT] = {0U};

    uint32_t start_tick;
    MotorId_t motor_id;

    if (sample_count == 0U)
    {
        return false;
    }

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        last_sequence[motor_id] =
            bsp_current_motor[motor_id].sample_sequence;
    }

    start_tick = HAL_GetTick();

    while ((collected_samples[MOTOR_ID_0] < sample_count) ||
           (collected_samples[MOTOR_ID_1] < sample_count))
    {
        for (motor_id = MOTOR_ID_0;
             motor_id < MOTOR_ID_COUNT;
             motor_id++)
        {
            uint16_t raw_a_count;
            uint16_t raw_b_count;

            if (collected_samples[motor_id] >= sample_count)
            {
                continue;
            }

            if (BspCurrent_TryReadRawSample(
                    &bsp_current_motor[motor_id],
                    &last_sequence[motor_id],
                    &raw_a_count,
                    &raw_b_count))
            {
                sum_a[motor_id] += raw_a_count;
                sum_b[motor_id] += raw_b_count;
                collected_samples[motor_id]++;
            }
        }

        if ((HAL_GetTick() - start_tick) >=
            BSP_CURRENT_CALIBRATION_TIMEOUT_MS)
        {
            return false;
        }
    }

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        bsp_current_motor[motor_id].config.offset_a_count =
            (float)sum_a[motor_id] /
            (float)sample_count;

        bsp_current_motor[motor_id].config.offset_b_count =
            (float)sum_b[motor_id] /
            (float)sample_count;
    }

    return true;
}

void HAL_ADCEx_InjectedConvCpltCallback(
    ADC_HandleTypeDef *hadc)
{
    BspCurrentMotorState_t *state = NULL;

    if (hadc == &hadc1)
    {
        state = &bsp_current_motor[MOTOR_ID_0];
    }
    else if (hadc == &hadc2)
    {
        state = &bsp_current_motor[MOTOR_ID_1];
    }

    if (state == NULL)
    {
        return;
    }

    state->raw_a_count =
        (uint16_t)HAL_ADCEx_InjectedGetValue(
            hadc,
            ADC_INJECTED_RANK_1);

    state->raw_b_count =
        (uint16_t)HAL_ADCEx_InjectedGetValue(
            hadc,
            ADC_INJECTED_RANK_2);

    /*
     * Increment only after both raw values are complete.
     */
    state->sample_sequence++;
}

static bool BspCurrent_IsValidMotorId(
    MotorId_t motor_id)
{
    return ((unsigned int)motor_id <
            (unsigned int)MOTOR_ID_COUNT);
}

static bool BspCurrent_TryReadRawSample(
    const BspCurrentMotorState_t *state,
    uint32_t *last_sequence,
    uint16_t *raw_a_count,
    uint16_t *raw_b_count)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    sequence_before = state->sample_sequence;

    if (sequence_before == *last_sequence)
    {
        return false;
    }

    *raw_a_count = state->raw_a_count;
    *raw_b_count = state->raw_b_count;

    sequence_after = state->sample_sequence;

    if (sequence_before != sequence_after)
    {
        return false;
    }

    *last_sequence = sequence_before;

    return true;
}
