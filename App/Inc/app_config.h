#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include "bsp_current.h"
#include "bsp_encoder.h"
#include "dual_motor.h"

/* Read-only application configuration accessors. */

/**
 * @brief Returns the application-level dual-motor configuration.
 *
 * @return Read-only configuration shared by the application initializer.
 */
const DualMotorConfig_t *AppConfig_GetDualMotorConfig(void);

/**
 * @brief Returns the application-level current-sensing configuration.
 */
const BspCurrentConfig_t *AppConfig_GetCurrentConfig(void);

/**
 * @brief Returns the application-level encoder configuration.
 */
const BspEncoderConfig_t *AppConfig_GetEncoderConfig(void);

#endif /* APP_CONFIG_H_ */
