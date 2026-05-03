#include "imu_icm42688p.h"

#include "imu_manager_config.h"
#include "main.h"
#include "spi.h"
#include <math.h>
#include <string.h>

#define ICM42688P_WHO_AM_I_REG       0x75U
#define ICM42688P_WHO_AM_I_VALUE     0x47U
#define ICM42688P_REG_BANK_SEL       0x76U
#define ICM42688P_DEVICE_CONFIG      0x11U
#define ICM42688P_PWR_MGMT0          0x4EU
#define ICM42688P_GYRO_CONFIG0       0x4FU
#define ICM42688P_ACCEL_CONFIG0      0x50U
#define ICM42688P_TEMP_DATA1         0x1DU

#define ICM42688P_SPI_READ_BIT       0x80U
#define ICM42688P_SPI_WRITE_MASK     0x7FU
#define ICM42688P_BANK0              0x00U
#define ICM42688P_SOFT_RESET_CMD     0x01U
#define ICM42688P_PWR_LOW_NOISE      0x0FU
#define ICM42688P_GYRO_2000DPS_200HZ 0x07U
#define ICM42688P_ACCEL_16G_200HZ    0x07U
#define ICM42688P_RAD_PER_DEG        0.017453292519943295f

static uint8_t s_online = 0U;

static void ImuIcm42688p_CsLow(void);
static void ImuIcm42688p_CsHigh(void);
static int ImuIcm42688p_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
static int ImuIcm42688p_WriteReg(uint8_t reg, uint8_t value);
static int16_t ImuIcm42688p_ReadI16Be(const uint8_t *data);

static void ImuIcm42688p_CsLow(void)
{
    HAL_GPIO_WritePin(SPI_IMU_NSS2_GPIO_Port, SPI_IMU_NSS2_Pin, GPIO_PIN_RESET);
}

static void ImuIcm42688p_CsHigh(void)
{
    HAL_GPIO_WritePin(SPI_IMU_NSS2_GPIO_Port, SPI_IMU_NSS2_Pin, GPIO_PIN_SET);
}

static int ImuIcm42688p_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[15];
    uint8_t rx[15];

    if ((data == NULL) || (len == 0U) || (len > 14U))
    {
        return -1;
    }

    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | ICM42688P_SPI_READ_BIT);

    ImuIcm42688p_CsLow();
    if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 1U), IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        ImuIcm42688p_CsHigh();
        return -1;
    }
    ImuIcm42688p_CsHigh();

    memcpy(data, &rx[1], len);
    return 0;
}

static int ImuIcm42688p_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = (uint8_t)(reg & ICM42688P_SPI_WRITE_MASK);
    tx[1] = value;

    ImuIcm42688p_CsLow();
    if (HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        ImuIcm42688p_CsHigh();
        return -1;
    }
    ImuIcm42688p_CsHigh();

    return 0;
}

static int16_t ImuIcm42688p_ReadI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

int ImuIcm42688p_Init(void)
{
    uint8_t chip_id = 0U;

    s_online = 0U;
    ImuIcm42688p_CsHigh();
    HAL_Delay(10U);

    (void)ImuIcm42688p_WriteReg(ICM42688P_REG_BANK_SEL, ICM42688P_BANK0);
    (void)ImuIcm42688p_WriteReg(ICM42688P_DEVICE_CONFIG, ICM42688P_SOFT_RESET_CMD);
    HAL_Delay(50U);
    (void)ImuIcm42688p_WriteReg(ICM42688P_REG_BANK_SEL, ICM42688P_BANK0);

    if (ImuIcm42688p_ReadRegs(ICM42688P_WHO_AM_I_REG, &chip_id, 1U) != 0)
    {
        return -1;
    }
    if (chip_id != ICM42688P_WHO_AM_I_VALUE)
    {
        return -1;
    }

    /*
     * PWR_MGMT0 = accel/gyro low-noise mode。
     * CONFIG0 低 4 位 ODR 选择 200Hz，量程位保持 0：gyro ±2000dps，accel ±16g。
     * 不启用 FIFO/DRDY/INT，后续若需要更精确滤波带宽再按数据手册扩展。
     */
    if (ImuIcm42688p_WriteReg(ICM42688P_GYRO_CONFIG0, ICM42688P_GYRO_2000DPS_200HZ) != 0)
    {
        return -1;
    }
    if (ImuIcm42688p_WriteReg(ICM42688P_ACCEL_CONFIG0, ICM42688P_ACCEL_16G_200HZ) != 0)
    {
        return -1;
    }
    if (ImuIcm42688p_WriteReg(ICM42688P_PWR_MGMT0, ICM42688P_PWR_LOW_NOISE) != 0)
    {
        return -1;
    }
    HAL_Delay(10U);

    s_online = 1U;
    return 0;
}

int ImuIcm42688p_Read(SensorsImuData_t *out)
{
    uint8_t buf[14];
    float accel_scale;
    float gyro_scale;

    if ((out == NULL) || (s_online == 0U))
    {
        return -1;
    }

    if (ImuIcm42688p_ReadRegs(ICM42688P_TEMP_DATA1, buf, sizeof(buf)) != 0)
    {
        s_online = 0U;
        return -1;
    }

    memset(out, 0, sizeof(*out));
    out->imu_id = IMU_ID_ICM42688P;
    out->online = 1U;
    out->data_valid = 1U;
    out->lastUpdate_ms = HAL_GetTick();
    out->temp_raw = ImuIcm42688p_ReadI16Be(&buf[0]);
    out->ax_raw = ImuIcm42688p_ReadI16Be(&buf[2]);
    out->ay_raw = ImuIcm42688p_ReadI16Be(&buf[4]);
    out->az_raw = ImuIcm42688p_ReadI16Be(&buf[6]);
    out->gx_raw = ImuIcm42688p_ReadI16Be(&buf[8]);
    out->gy_raw = ImuIcm42688p_ReadI16Be(&buf[10]);
    out->gz_raw = ImuIcm42688p_ReadI16Be(&buf[12]);

    accel_scale = IMU_GRAVITY_MPS2 / 2048.0f;
    gyro_scale = ICM42688P_RAD_PER_DEG / 16.4f;
    out->ax_meas_mps2 = ((float)out->ax_raw) * accel_scale;
    out->ay_meas_mps2 = ((float)out->ay_raw) * accel_scale;
    out->az_meas_mps2 = ((float)out->az_raw) * accel_scale;
    out->gx_meas_radps = ((float)out->gx_raw) * gyro_scale;
    out->gy_meas_radps = ((float)out->gy_raw) * gyro_scale;
    out->gz_meas_radps = ((float)out->gz_raw) * gyro_scale;
    out->temperature_degC = ((float)out->temp_raw) / 132.48f + 25.0f;

    return (isfinite(out->ax_meas_mps2) && isfinite(out->temperature_degC)) ? 0 : -1;
}

uint8_t ImuIcm42688p_IsOnline(void)
{
    return s_online;
}
