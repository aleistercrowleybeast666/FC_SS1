#include "imu_bmi088.h"

#include "imu_manager_config.h"
#include "main.h"
#include "spi.h"
#include <math.h>
#include <string.h>

#define BMI088_ACC_CHIP_ID_REG       0x00U
#define BMI088_ACC_CHIP_ID_VALUE     0x1EU
#define BMI088_ACC_DATA_REG          0x12U
#define BMI088_ACC_CONF_REG          0x40U
#define BMI088_ACC_RANGE_REG         0x41U
#define BMI088_ACC_PWR_CONF_REG      0x7CU
#define BMI088_ACC_PWR_CTRL_REG      0x7DU
#define BMI088_ACC_SOFTRESET_REG     0x7EU

#define BMI088_GYRO_CHIP_ID_REG      0x00U
#define BMI088_GYRO_CHIP_ID_VALUE    0x0FU
#define BMI088_GYRO_DATA_REG         0x02U
#define BMI088_GYRO_RANGE_REG        0x0FU
#define BMI088_GYRO_BANDWIDTH_REG    0x10U
#define BMI088_GYRO_LPM1_REG         0x11U
#define BMI088_GYRO_SOFTRESET_REG    0x14U

#define BMI088_SPI_READ_BIT          0x80U
#define BMI088_SPI_WRITE_MASK        0x7FU
#define BMI088_ACC_RANGE_24G         0x03U
#define BMI088_ACC_CONF_200HZ        0xA9U
#define BMI088_ACC_PWR_ACTIVE        0x00U
#define BMI088_ACC_PWR_ENABLE        0x04U
#define BMI088_GYRO_RANGE_2000DPS    0x00U
#define BMI088_GYRO_BW_200HZ_64HZ    0x06U
#define BMI088_GYRO_NORMAL_MODE      0x00U
#define BMI088_SOFTRESET_CMD         0xB6U
#define BMI088_RAD_PER_DEG           0.017453292519943295f

static uint8_t s_online = 0U;

static void ImuBmi088_AccCsLow(void);
static void ImuBmi088_AccCsHigh(void);
static void ImuBmi088_GyroCsLow(void);
static void ImuBmi088_GyroCsHigh(void);
static int ImuBmi088_AccRead(uint8_t reg, uint8_t *data, uint16_t len);
static int ImuBmi088_GyroRead(uint8_t reg, uint8_t *data, uint16_t len);
static int ImuBmi088_AccWrite(uint8_t reg, uint8_t value);
static int ImuBmi088_GyroWrite(uint8_t reg, uint8_t value);
static int16_t ImuBmi088_ReadI16Le(const uint8_t *data);

static void ImuBmi088_AccCsLow(void)
{
    HAL_GPIO_WritePin(SPI_IMU_NSS1A_GPIO_Port, SPI_IMU_NSS1A_Pin, GPIO_PIN_RESET);
}

static void ImuBmi088_AccCsHigh(void)
{
    HAL_GPIO_WritePin(SPI_IMU_NSS1A_GPIO_Port, SPI_IMU_NSS1A_Pin, GPIO_PIN_SET);
}

static void ImuBmi088_GyroCsLow(void)
{
    HAL_GPIO_WritePin(SPI_IMU_NSS1G_GPIO_Port, SPI_IMU_NSS1G_Pin, GPIO_PIN_RESET);
}

static void ImuBmi088_GyroCsHigh(void)
{
    HAL_GPIO_WritePin(SPI_IMU_NSS1G_GPIO_Port, SPI_IMU_NSS1G_Pin, GPIO_PIN_SET);
}

static int ImuBmi088_AccRead(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[8];
    uint8_t rx[8];

    if ((data == NULL) || (len == 0U) || (len > 6U))
    {
        return -1;
    }

    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | BMI088_SPI_READ_BIT);

    ImuBmi088_AccCsLow();
    if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 2U), IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        ImuBmi088_AccCsHigh();
        return -1;
    }
    ImuBmi088_AccCsHigh();

    memcpy(data, &rx[2], len);
    return 0;
}

static int ImuBmi088_GyroRead(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[7];
    uint8_t rx[7];

    if ((data == NULL) || (len == 0U) || (len > 6U))
    {
        return -1;
    }

    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | BMI088_SPI_READ_BIT);

    ImuBmi088_GyroCsLow();
    if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 1U), IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        ImuBmi088_GyroCsHigh();
        return -1;
    }
    ImuBmi088_GyroCsHigh();

    memcpy(data, &rx[1], len);
    return 0;
}

static int ImuBmi088_AccWrite(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = (uint8_t)(reg & BMI088_SPI_WRITE_MASK);
    tx[1] = value;

    ImuBmi088_AccCsLow();
    if (HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        ImuBmi088_AccCsHigh();
        return -1;
    }
    ImuBmi088_AccCsHigh();

    return 0;
}

static int ImuBmi088_GyroWrite(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = (uint8_t)(reg & BMI088_SPI_WRITE_MASK);
    tx[1] = value;

    ImuBmi088_GyroCsLow();
    if (HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        ImuBmi088_GyroCsHigh();
        return -1;
    }
    ImuBmi088_GyroCsHigh();

    return 0;
}

static int16_t ImuBmi088_ReadI16Le(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

int ImuBmi088_Init(void)
{
    uint8_t chip_id = 0U;

    s_online = 0U;
    ImuBmi088_AccCsHigh();
    ImuBmi088_GyroCsHigh();
    HAL_Delay(10U);

    (void)ImuBmi088_AccRead(BMI088_ACC_CHIP_ID_REG, &chip_id, 1U);
    if (ImuBmi088_AccRead(BMI088_ACC_CHIP_ID_REG, &chip_id, 1U) != 0)
    {
        return -1;
    }
    if (chip_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return -1;
    }

    if (ImuBmi088_GyroRead(BMI088_GYRO_CHIP_ID_REG, &chip_id, 1U) != 0)
    {
        return -1;
    }
    if (chip_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return -1;
    }

    (void)ImuBmi088_AccWrite(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_CMD);
    (void)ImuBmi088_GyroWrite(BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_CMD);
    HAL_Delay(50U);

    if (ImuBmi088_AccWrite(BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_PWR_ENABLE) != 0)
    {
        return -1;
    }
    HAL_Delay(5U);
    if (ImuBmi088_AccWrite(BMI088_ACC_PWR_CONF_REG, BMI088_ACC_PWR_ACTIVE) != 0)
    {
        return -1;
    }
    if (ImuBmi088_AccWrite(BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_24G) != 0)
    {
        return -1;
    }
    if (ImuBmi088_AccWrite(BMI088_ACC_CONF_REG, BMI088_ACC_CONF_200HZ) != 0)
    {
        return -1;
    }

    if (ImuBmi088_GyroWrite(BMI088_GYRO_LPM1_REG, BMI088_GYRO_NORMAL_MODE) != 0)
    {
        return -1;
    }
    if (ImuBmi088_GyroWrite(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS) != 0)
    {
        return -1;
    }
    /*
     * GYRO_BANDWIDTH 选择 200Hz ODR / 64Hz filter bandwidth。
     * 第一版只做轮询短读，不启用 FIFO/DRDY。
     */
    if (ImuBmi088_GyroWrite(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BW_200HZ_64HZ) != 0)
    {
        return -1;
    }

    s_online = 1U;
    return 0;
}

int ImuBmi088_Read(SensorsImuData_t *out)
{
    uint8_t acc_buf[6];
    uint8_t gyro_buf[6];
    float accel_scale;
    float gyro_scale;

    if ((out == NULL) || (s_online == 0U))
    {
        return -1;
    }

    if (ImuBmi088_AccRead(BMI088_ACC_DATA_REG, acc_buf, sizeof(acc_buf)) != 0)
    {
        s_online = 0U;
        return -1;
    }
    if (ImuBmi088_GyroRead(BMI088_GYRO_DATA_REG, gyro_buf, sizeof(gyro_buf)) != 0)
    {
        s_online = 0U;
        return -1;
    }

    memset(out, 0, sizeof(*out));
    out->imu_id = IMU_ID_BMI088;
    out->online = 1U;
    out->data_valid = 1U;
    out->lastUpdate_ms = HAL_GetTick();
    out->ax_raw = ImuBmi088_ReadI16Le(&acc_buf[0]);
    out->ay_raw = ImuBmi088_ReadI16Le(&acc_buf[2]);
    out->az_raw = ImuBmi088_ReadI16Le(&acc_buf[4]);
    out->gx_raw = ImuBmi088_ReadI16Le(&gyro_buf[0]);
    out->gy_raw = ImuBmi088_ReadI16Le(&gyro_buf[2]);
    out->gz_raw = ImuBmi088_ReadI16Le(&gyro_buf[4]);

    accel_scale = ((float)IMU_ACCEL_RANGE_G * IMU_GRAVITY_MPS2) / 32768.0f;
    gyro_scale = ((float)IMU_GYRO_RANGE_DPS * BMI088_RAD_PER_DEG) / 32768.0f;
    out->ax_meas_mps2 = ((float)out->ax_raw) * accel_scale;
    out->ay_meas_mps2 = ((float)out->ay_raw) * accel_scale;
    out->az_meas_mps2 = ((float)out->az_raw) * accel_scale;
    out->gx_meas_radps = ((float)out->gx_raw) * gyro_scale;
    out->gy_meas_radps = ((float)out->gy_raw) * gyro_scale;
    out->gz_meas_radps = ((float)out->gz_raw) * gyro_scale;
    /* TODO: BMI088 温度寄存器格式后续按实测补充，第一版先保留为 0。 */
    out->temp_raw = 0;
    out->temperature_degC = 0.0f;

    return (isfinite(out->ax_meas_mps2) && isfinite(out->gx_meas_radps)) ? 0 : -1;
}

uint8_t ImuBmi088_IsOnline(void)
{
    return s_online;
}
