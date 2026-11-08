#ifndef FOC_PID_H_
#define FOC_PID_H_

/**
 * @brief Tunable parameters of a PI controller.
 */
typedef struct
{
    float kp;
    float ki;
    float output_min;
    float output_max;
} FocPiConfig_t;

/**
 * @brief Runtime state of one PI controller instance.
 */
typedef struct
{
    FocPiConfig_t config;

    /* Integral term in output unit. */
    float integrator;

    /* Output from the most recent control step. */
    float last_output;
} FocPi_t;

/**
 * @brief Initializes a PI controller instance.
 * @param controller Target PI controller. Must not be NULL.
 * @param config PI parameters to copy. Must not be NULL.
 */
void FocPi_Init(FocPi_t *controller, const FocPiConfig_t *config);

/**
 * @brief Clears the PI runtime state and keeps its configuration.
 * @param controller Target PI controller. Must not be NULL.
 */
void FocPi_Reset(FocPi_t *controller);

/**
 * @brief Executes one discrete PI control step.
 * @param controller Target PI controller. Must not be NULL.
 * @param reference Desired input value.
 * @param feedback Measured input value.
 * @param sample_time_s Control period in seconds and must be greater than zero.
 * @return Limited controller output.
 */
float FocPi_Update(FocPi_t *controller,
                   float reference,
                   float feedback,
                   float sample_time_s);


#endif /* FOC_PID_H_ */
