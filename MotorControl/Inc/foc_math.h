#ifndef FOC_MATH_H_
#define FOC_MATH_H_

#include "motor_types.h"

/**
 * @brief Restricts a value to an inclusive range.
 * @param lower_limit Lower bound. Must not exceed upper_limit.
 * @param upper_limit Upper bound.
 * @return Limited value.
 */
float FocMath_Clamp(float value, float lower_limit, float upper_limit);

/**
 * @brief Calculates sine and cosine of an electrical angle.
 * @param electrical_angle_rad Electrical angle in radians.
 * @return Sine and cosine values for the given angle.
 */
FocSinCos_t FocMath_GetSinCos(float electrical_angle_rad);

/**
 * @brief Converts three-phase current to stationary alpha-beta current.
 * @note Assumes phase_a_A + phase_b_A + phase_c_A equals zero.
 */
AlphaBetaCurrent_t FocMath_Clarke(PhaseCurrent_t phase_current);

/**
 * @brief Converts stationary alpha-beta current to rotating d-q current.
 */
DqCurrent_t FocMath_Park(AlphaBetaCurrent_t alpha_beta_current,
                          FocSinCos_t sin_cos);

/**
 * @brief Converts rotating d-q voltage to stationary alpha-beta voltage.
 */
AlphaBetaVoltage_t FocMath_InversePark(DqVoltage_t dq_voltage,
                                        FocSinCos_t sin_cos);

/**
 * @param dc_bus_voltage_V DC bus voltage in volt.
 * @return PWM duties limited to the range [0.0f, 1.0f].
 * @note A non-positive bus voltage returns neutral duty on all phases.
 */
PwmDuty_t FocMath_Svpwm(AlphaBetaVoltage_t alpha_beta_voltage,
                         float dc_bus_voltage_V);

    
#endif /* FOC_MATH_H_ */
