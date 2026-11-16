#ifndef BSP_ENCODER_H_
#define BSP_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

#include "motor_types.h"



/* One complete encoder measurement result. */
typedef struct
{
    uint16_t raw_angle;              /* 14-bit AS5047P angle value */
    float mechanical_angle_rad;      /* Mechanical angle in radians */
    bool parity_ok;                  /* SPI response parity status */
    bool command_error;              /* AS5047P error flag */
} BspEncoderSample_t;

/* Per-motor encoder configuration. */
typedef struct
{
    uint16_t pole_pairs;             /* Motor pole-pair count */
    float electrical_offset_rad;     /* Electrical zero offset */
    bool invert_direction;            /* Reverse angle direction if needed */
} BspEncoderMotorConfig_t;

/* Shared configuration for both motors. */
typedef struct
{
    BspEncoderMotorConfig_t motor_config[MOTOR_ID_COUNT];
} BspEncoderConfig_t;

/* AS5047P volatile register used for compensated angle reads. */
#define BSP_ENCODER_REG_ANGLECOM (0x3FFFU)

/* Initialize encoder configuration and internal state. */
void BspEncoder_Init(const BspEncoderConfig_t *config);

/* Read one AS5047P sample from the selected motor. */
bool BspEncoder_ReadSample(
    MotorId_t motor_id,
    BspEncoderSample_t *sample);

/* Convert the encoder angle to FOC electrical angle. */
bool BspEncoder_GetElectricalAngle(
    MotorId_t motor_id,
    float *electrical_angle_rad);

bool BspEncoder_ComputeElectricalAngle(
    float mechanical_angle_rad,
    const BspEncoderMotorConfig_t *config,
    float *electrical_angle_rad);

bool BspEncoder_ConvertRawAngle(
    uint16_t raw_angle,
    float *mechanical_angle_rad);

bool BspEncoder_IsEvenParity(uint16_t frame);

/* Build a 16-bit AS5047P read command with even parity. */
bool BspEncoder_BuildReadCommand(
    uint16_t register_address,
    uint16_t *command);

/* Parse the response returned by the following NOP frame. */
bool BspEncoder_ParseAngleResponse(
    uint16_t response,
    uint16_t *raw_angle,
    bool *command_error);

#endif /* BSP_ENCODER_H_ */
