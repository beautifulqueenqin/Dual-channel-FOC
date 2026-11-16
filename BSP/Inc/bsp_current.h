#ifndef BSP_CURRENT_H_
#define BSP_CURRENT_H_

#include <stdbool.h>
#include <stdint.h>

#include "motor_types.h"

/**
 * @brief Conversion parameters for one motor current channel.
 */
typedef struct
{
    /*
     * Motor current represented by one ADC count.
     */
    float ampere_per_count;

    /*
     * Zero-current ADC offsets measured during calibration.
     */
    float offset_a_count;
    float offset_b_count;
} BspCurrentMotorConfig_t;

/**
 * @brief Current-sensing configuration for both motors.
 */
typedef struct
{
    BspCurrentMotorConfig_t motor_config[MOTOR_ID_COUNT];
} BspCurrentConfig_t;

/**
 * @brief Initializes current-sensing state and calibration offsets.
 */
void BspCurrent_Init(
    const BspCurrentConfig_t *config);

/**
 * @brief Starts timer-triggered injected ADC conversions.
 *
 * @return True when both ADC channels start successfully.
 */
bool BspCurrent_Start(void);

/**
 * @brief Stops injected ADC conversions.
 */
void BspCurrent_Stop(void);

/**
 * @brief Reads the latest reconstructed three-phase current.
 *
 * @return False when the motor ID is invalid or no sample is ready.
 */
bool BspCurrent_GetPhaseCurrent(
    MotorId_t motor_id,
    PhaseCurrent_t *phase_current);

/**
 * @brief Reports whether a fresh ADC sample is available.
 */
bool BspCurrent_IsSampleReady(
    MotorId_t motor_id);

/**
 * @brief Calibrates ADC offsets while no motor current is flowing.
 *
 * @param sample_count Number of valid injected samples to average.
 *
 * @return True when calibration completes successfully.
 *
 * @note The power stage must remain disabled during calibration.
 * @note ADC injected conversion and PWM trigger must already be running.
 */
bool BspCurrent_CalibrateOffsets(uint32_t sample_count);

#endif /* BSP_CURRENT_H_ */
