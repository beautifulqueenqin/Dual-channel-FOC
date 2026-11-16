#ifndef BSP_PWM_H_
#define BSP_PWM_H_

#include <stdbool.h>

#include "motor_types.h"

/**
 * @brief Initializes PWM compare values to neutral duty.
 */
void BspPwm_Init(void);

/**
 * @brief Starts main and complementary PWM outputs.
 *
 * @return True when all PWM channels start successfully.
 */
bool BspPwm_Start(void);

/**
 * @brief Stops all PWM outputs.
 */
void BspPwm_Stop(void);

/**
 * @brief Writes normalized three-phase duty to one motor timer.
 *
 * @param motor_id Motor channel selector.
 * @param pwm_duty Normalized duty values in the range [0.0, 1.0].
 *
 * @return True when the motor ID and duty pointer are valid.
 */
bool BspPwm_SetMotorDuty(
    MotorId_t motor_id,
    const PwmDuty_t *pwm_duty);

#endif /* BSP_PWM_H_ */
