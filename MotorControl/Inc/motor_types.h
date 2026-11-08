#ifndef MOTOR_TYPES_H_
#define MOTOR_TYPES_H_

/**
 * @brief Identifies a physical motor channel on the controller board.
 */
typedef enum
{
    MOTOR_ID_0 = 0,
    MOTOR_ID_1,
    MOTOR_ID_COUNT
} MotorId_t;

/**
 * @brief Three-phase motor current in ampere.
 * @note Positive current flows from the inverter to the motor phase.
 */
typedef struct
{
    float phase_a_A;
    float phase_b_A;
    float phase_c_A;
} PhaseCurrent_t;

/**
 * @brief Stationary-frame current after Clarke transformation.
 */
typedef struct
{
    float alpha_A;
    float beta_A;
} AlphaBetaCurrent_t;

/**
 * @brief Stationary-frame voltage in alpha-beta coordinates.
 */
typedef struct
{
    float alpha_V;
    float beta_V;
} AlphaBetaVoltage_t;

/**
 * @brief Sine and cosine of an electrical angle.
 */
typedef struct
{
    float sin_theta;
    float cos_theta;
} FocSinCos_t;

/**
 * @brief Rotating-frame current after Park transformation.
 */
typedef struct
{
    float d_A;
    float q_A;
} DqCurrent_t;

/**
 * @brief Voltage command in the rotating d-q reference frame.
 */
typedef struct
{
    float d_V;
    float q_V;
} DqVoltage_t;

/**
 * @brief Normalized three-phase PWM duty cycle.
 * @note Each value must stay within the range [0.0f, 1.0f].
 */
typedef struct
{
    float phase_a;
    float phase_b;
    float phase_c;
} PwmDuty_t;

#endif /* MOTOR_TYPES_H_ */
