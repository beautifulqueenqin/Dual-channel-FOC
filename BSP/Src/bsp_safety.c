/* Board-specific fault and power-stage gate control. */
#include "bsp_safety.h"

#include "main.h"

/*
 * Written by the nFAULT interrupt path and read by normal application code.
 */
static volatile bool bsp_safety_fault_latched;

void BspSafety_Init(void)
{
    /*
     * Never enable the power stage during initialization.
     */
    BspSafety_DisablePowerStage();

    /*
     * A fault already present at startup must remain latched.
     */
    bsp_safety_fault_latched =
        BspSafety_IsFaultInputActive();
}

void BspSafety_DisablePowerStage(void)
{
    HAL_GPIO_WritePin(
        EN_GATE_GPIO_Port,
        EN_GATE_Pin,
        GPIO_PIN_RESET);
}

bool BspSafety_IsFaultInputActive(void)
{
    return (HAL_GPIO_ReadPin(
                nFAULT_GPIO_Port,
                nFAULT_Pin) == GPIO_PIN_RESET);
}

bool BspSafety_TryEnablePowerStage(void)
{
    /*
     * A hardware fault always has higher priority
     * than an enable request.
     */
    if (BspSafety_IsFaultInputActive())
    {
        bsp_safety_fault_latched = true;
    }

    /*
     * Never enable the power stage while a fault is latched.
     */
    if (bsp_safety_fault_latched)
    {
        BspSafety_DisablePowerStage();
        return false;
    }

    /*
     * The hardware safety conditions are satisfied.
     */
    HAL_GPIO_WritePin(
        EN_GATE_GPIO_Port,
        EN_GATE_Pin,
        GPIO_PIN_SET);

    return true;
}

bool BspSafety_IsFaultLatched(void)
{
    return bsp_safety_fault_latched;
}

bool BspSafety_ClearFaultLatch(void)
{
    /*
     * Keep the power stage disabled during fault recovery.
     */
    BspSafety_DisablePowerStage();

    /*
     * The hardware fault must disappear before clearing the latch.
     */
    if (BspSafety_IsFaultInputActive())
    {
        bsp_safety_fault_latched = true;
        return false;
    }

    /*
     * Clear only the software latch.
     * Power-stage enabling is a separate operation.
     */
    bsp_safety_fault_latched = false;

    return true;
}

void BspSafety_HandleFaultInterrupt(void)
{
    /*
     * Latch the fault before changing the gate-enable output.
     */
    bsp_safety_fault_latched = true;

    /*
     * Disable the power stage immediately.
     */
    BspSafety_DisablePowerStage();
}

/**
 * @brief HAL callback for GPIO external interrupts.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == nFAULT_Pin)
    {
        BspSafety_HandleFaultInterrupt();
    }
}
