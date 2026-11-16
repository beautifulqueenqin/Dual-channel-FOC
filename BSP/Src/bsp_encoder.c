/* Encoder SPI hardware adaptation. */
#include "bsp_encoder.h"

#include <math.h>
#include <stddef.h>

#include "gpio.h"
#include "spi.h"

#define BSP_ENCODER_RAW_MASK       (0x3FFFU)
#define BSP_ENCODER_RAW_RESOLUTION (16384.0f)
#define BSP_ENCODER_TWO_PI         (6.28318530717958647692f)

#define BSP_ENCODER_READ_BIT       (0x4000U)
#define BSP_ENCODER_PARITY_BIT     (0x8000U)
#define BSP_ENCODER_ADDRESS_MASK   (0x3FFFU)
#define BSP_ENCODER_ERROR_FLAG     (0x4000U)
#define BSP_ENCODER_DATA_MASK      (0x3FFFU)
#define BSP_ENCODER_NOP_FRAME      (0x0000U)
#define BSP_ENCODER_SPI_TIMEOUT_MS (10U)

typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} BspEncoderBus_t;

typedef struct
{
    BspEncoderMotorConfig_t config;
} BspEncoderMotorState_t;

static BspEncoderMotorState_t
    bsp_encoder_motor[MOTOR_ID_COUNT];

static bool BspEncoder_IsValidMotorId(
    MotorId_t motor_id);

static bool BspEncoder_GetBus(
    MotorId_t motor_id,
    BspEncoderBus_t *bus);

static bool BspEncoder_TransferFrame(
    const BspEncoderBus_t *bus,
    uint16_t tx_frame,
    uint16_t *rx_frame);

void BspEncoder_Init(
    const BspEncoderConfig_t *config)
{
    MotorId_t motor_id;
    BspEncoderBus_t bus;

    for (motor_id = MOTOR_ID_0;
         motor_id < MOTOR_ID_COUNT;
         motor_id++)
    {
        if (config != NULL)
        {
            bsp_encoder_motor[motor_id].config =
                config->motor_config[motor_id];
        }
        else
        {
            bsp_encoder_motor[motor_id].config =
                (BspEncoderMotorConfig_t){0};
        }

        /* Keep every encoder chip deselected while idle. */
        if (BspEncoder_GetBus(motor_id, &bus))
        {
            HAL_GPIO_WritePin(
                bus.cs_port,
                bus.cs_pin,
                GPIO_PIN_SET);
        }
    }
}

bool BspEncoder_ReadSample(
    MotorId_t motor_id,
    BspEncoderSample_t *sample)
{
    BspEncoderBus_t bus;
    uint16_t read_command;
    uint16_t ignored_response;
    uint16_t response;
    uint16_t raw_angle;
    float mechanical_angle_rad;
    bool command_error;

    if ((sample == NULL) ||
        (!BspEncoder_IsValidMotorId(motor_id)))
    {
        return false;
    }

    *sample = (BspEncoderSample_t){0};

    if (!BspEncoder_GetBus(motor_id, &bus))
    {
        return false;
    }

    if (!BspEncoder_BuildReadCommand(
            BSP_ENCODER_REG_ANGLECOM,
            &read_command))
    {
        return false;
    }

    /* The response to the read command is intentionally discarded. */
    if (!BspEncoder_TransferFrame(
            &bus,
            read_command,
            &ignored_response))
    {
        return false;
    }

    /* The following NOP frame receives the previous command response. */
    if (!BspEncoder_TransferFrame(
            &bus,
            BSP_ENCODER_NOP_FRAME,
            &response))
    {
        return false;
    }

    sample->parity_ok =
        BspEncoder_IsEvenParity(response);

    if (!BspEncoder_ParseAngleResponse(
            response,
            &raw_angle,
            &command_error))
    {
        sample->command_error = command_error;
        return false;
    }

    sample->command_error = command_error;

    if (!BspEncoder_ConvertRawAngle(
            raw_angle,
            &mechanical_angle_rad))
    {
        return false;
    }

    sample->raw_angle = raw_angle;
    sample->mechanical_angle_rad = mechanical_angle_rad;

    return true;
}

bool BspEncoder_GetElectricalAngle(
    MotorId_t motor_id,
    float *electrical_angle_rad)
{
    BspEncoderSample_t sample;

    if (electrical_angle_rad == NULL)
    {
        return false;
    }

    if (!BspEncoder_ReadSample(motor_id, &sample))
    {
        return false;
    }

    return BspEncoder_ComputeElectricalAngle(
        sample.mechanical_angle_rad,
        &bsp_encoder_motor[motor_id].config,
        electrical_angle_rad);
}

bool BspEncoder_ComputeElectricalAngle(
    float mechanical_angle_rad,
    const BspEncoderMotorConfig_t *config,
    float *electrical_angle_rad)
{
    float electrical_angle;

    if ((config == NULL) ||
        (electrical_angle_rad == NULL) ||
        (config->pole_pairs == 0U) ||
        !isfinite(mechanical_angle_rad))
    {
        return false;
    }

    /* Reverse the mechanical direction when required. */
    if (config->invert_direction)
    {
        mechanical_angle_rad = -mechanical_angle_rad;
    }

    /*
     * Electrical angle:
     *
     * theta_e = pole_pairs * theta_m + offset
     */
    electrical_angle =
        mechanical_angle_rad * (float)config->pole_pairs +
        config->electrical_offset_rad;

    /* Wrap the angle into [0, 2*pi). */
    electrical_angle = fmodf(
        electrical_angle,
        BSP_ENCODER_TWO_PI);

    if (electrical_angle < 0.0f)
    {
        electrical_angle += BSP_ENCODER_TWO_PI;
    }

    *electrical_angle_rad = electrical_angle;

    return true;
}

bool BspEncoder_ConvertRawAngle(
    uint16_t raw_angle,
    float *mechanical_angle_rad)
{
    if ((mechanical_angle_rad == NULL) ||
        (raw_angle > BSP_ENCODER_RAW_MASK))
    {
        return false;
    }

    /*
     * Mechanical angle:
     *
     * theta_m = raw_angle * 2*pi / 16384
     */
    *mechanical_angle_rad =
        ((float)raw_angle * BSP_ENCODER_TWO_PI) / BSP_ENCODER_RAW_RESOLUTION;

    return true;
}

bool BspEncoder_IsEvenParity(uint16_t frame)
{
    uint8_t one_count = 0U;
    uint8_t bit_index;

    /*
     * Count the number of '1' bits in the complete 16-bit frame.
     * AS5047P uses even parity.
     */
    for (bit_index = 0U; bit_index < 16U; ++bit_index)
    {
        one_count += (uint8_t)(frame & 0x0001U);
        frame >>= 1U;
    }

    return ((one_count % 2U) == 0U);
}

bool BspEncoder_BuildReadCommand(
    uint16_t register_address,
    uint16_t *command)
{
    uint16_t frame;

    if ((command == NULL) ||
        (register_address > BSP_ENCODER_ADDRESS_MASK))
    {
        return false;
    }

    /*
     * Frame format:
     *
     * bit 15    : even parity
     * bit 14    : read command
     * bits 13:0 : register address
     */
    frame = (uint16_t)(BSP_ENCODER_READ_BIT |
                       register_address);

    if (!BspEncoder_IsEvenParity(frame))
    {
        frame |= BSP_ENCODER_PARITY_BIT;
    }

    *command = frame;

    return true;
}

bool BspEncoder_ParseAngleResponse(
    uint16_t response,
    uint16_t *raw_angle,
    bool *command_error)
{
    bool parity_ok;
    bool error_flag;

    if ((raw_angle == NULL) ||
        (command_error == NULL))
    {
        return false;
    }

    parity_ok = BspEncoder_IsEvenParity(response);
    error_flag =
        ((response & BSP_ENCODER_ERROR_FLAG) != 0U);

    *command_error = error_flag;
    *raw_angle = (uint16_t)(response & BSP_ENCODER_DATA_MASK);

    /*
     * A response is valid only when:
     * 1. Even parity is correct.
     * 2. The previous command caused no error.
     */
    return parity_ok && !error_flag;
}

static bool BspEncoder_IsValidMotorId(
    MotorId_t motor_id)
{
    return ((unsigned int)motor_id <
            (unsigned int)MOTOR_ID_COUNT);
}

static bool BspEncoder_GetBus(
    MotorId_t motor_id,
    BspEncoderBus_t *bus)
{
    if (bus == NULL)
    {
        return false;
    }

    switch (motor_id)
    {
        case MOTOR_ID_0:
            bus->spi = &hspi1;
            bus->cs_port = M0_ENC_CS_GPIO_Port;
            bus->cs_pin = M0_ENC_CS_Pin;
            return true;

        case MOTOR_ID_1:
            bus->spi = &hspi3;
            bus->cs_port = M1_ENC_CS_GPIO_Port;
            bus->cs_pin = M1_ENC_CS_Pin;
            return true;

        default:
            return false;
    }
}

static bool BspEncoder_TransferFrame(
    const BspEncoderBus_t *bus,
    uint16_t tx_frame,
    uint16_t *rx_frame)
{
    HAL_StatusTypeDef status;

    if ((bus == NULL) ||
        (bus->spi == NULL) ||
        (bus->cs_port == NULL) ||
        (rx_frame == NULL))
    {
        return false;
    }

    HAL_GPIO_WritePin(
        bus->cs_port,
        bus->cs_pin,
        GPIO_PIN_RESET);

    status = HAL_SPI_TransmitReceive(
        bus->spi,
        (uint8_t *)&tx_frame,
        (uint8_t *)rx_frame,
        1U,
        BSP_ENCODER_SPI_TIMEOUT_MS);

    /* CS must return high even when the transfer fails. */
    HAL_GPIO_WritePin(
        bus->cs_port,
        bus->cs_pin,
        GPIO_PIN_SET);

    return (status == HAL_OK);
}
