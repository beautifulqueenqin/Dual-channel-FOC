#ifndef BSP_SAFETY_H_
#define BSP_SAFETY_H_

#include <stdbool.h>

/**
 * @brief Initializes the board safety layer in the disabled state.
 *
 * @note GPIO initialization is performed by STM32CubeMX before this call.
 */
void BspSafety_Init(void);

/**
 * @brief Immediately disables the power stage through EN_GATE.
 *
 * @note This function does not clear a latched fault.
 */
void BspSafety_DisablePowerStage(void);

/**
 * @brief Enables the power stage only when the safety state allows it.
 *
 * @return True when EN_GATE is enabled.
 * @note This function does not start PWM generation.
 */
bool BspSafety_TryEnablePowerStage(void);

/**
 * @brief Reports whether the active-low nFAULT input is asserted.
 */
bool BspSafety_IsFaultInputActive(void);

/**
 * @brief Reports whether a software fault latch is active.
 */
bool BspSafety_IsFaultLatched(void);

/**
 * @brief Clears the software fault latch when the nFAULT input is inactive.
 *
 * @return True when the fault latch is cleared.
 * @note The power stage remains disabled after this call.
 */
bool BspSafety_ClearFaultLatch(void);

/**
 * @brief Handles an nFAULT interrupt and immediately disables the power stage.
 */
void BspSafety_HandleFaultInterrupt(void);

#endif /* BSP_SAFETY_H_ */
