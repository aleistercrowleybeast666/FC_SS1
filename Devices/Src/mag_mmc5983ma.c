#include "mag_mmc5983ma.h"

#include "i2c.h"
#include "sensor_config.h"
#include <string.h>

#define MMC5983MA_REG_XOUT0         0x00U
#define MMC5983MA_REG_STATUS        0x08U
#define MMC5983MA_REG_CTRL0         0x09U
#define MMC5983MA_REG_CTRL1         0x0AU
#define MMC5983MA_REG_PRODUCT_ID    0x2FU
#define MMC5983MA_PRODUCT_ID        0x30U
#define MMC5983MA_I2C_DEV_ADDR      (MMC5983MA_I2C_ADDR << 1)

#define MMC5983MA_STATUS_MEAS_M_DONE    0x01U
#define MMC5983MA_CTRL0_TM_M            0x01U
#define MMC5983MA_CTRL0_SET             0x08U
#define MMC5983MA_CTRL0_RESET           0x10U
#define MMC5983MA_CTRL0_AUTO_SR_EN      0x20U
#define MMC5983MA_CTRL1_SW_RST          0x80U

#define MMC5983MA_RAW_CENTER        131072L
#define MMC5983MA_UT_PER_LSB        0.00625f
#define MMC5983MA_MEAS_TIMEOUT_MS   15U

static SensorsMagData_t s_data;
static uint8_t s_initialized = 0U;
static uint8_t s_measure_pending = 0U;
static uint32_t s_measure_start_ms = 0U;

static HAL_StatusTypeDef Mag_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef Mag_WriteReg(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef Mag_Set(void);
static HAL_StatusTypeDef Mag_Reset(void);
static uint32_t Mag_ReadRaw18(uint8_t msb, uint8_t lsb, uint8_t extra_bits);
static void Mag_ParseRaw(const uint8_t *reg_data, uint32_t now_ms);

static HAL_StatusTypeDef Mag_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            MMC5983MA_I2C_DEV_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            len,
                            SENSOR_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Mag_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c2,
                             MMC5983MA_I2C_DEV_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1U,
                             SENSOR_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Mag_Set(void)
{
    return Mag_WriteReg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_SET);
}

static HAL_StatusTypeDef Mag_Reset(void)
{
    return Mag_WriteReg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_RESET);
}

static uint32_t Mag_ReadRaw18(uint8_t msb, uint8_t lsb, uint8_t extra_bits)
{
    return ((uint32_t)msb << 10) | ((uint32_t)lsb << 2) | (extra_bits & 0x03U);
}

static void Mag_ParseRaw(const uint8_t *reg_data, uint32_t now_ms)
{
    uint32_t raw_x = Mag_ReadRaw18(reg_data[0], reg_data[1], (uint8_t)(reg_data[6] >> 6));
    uint32_t raw_y = Mag_ReadRaw18(reg_data[2], reg_data[3], (uint8_t)(reg_data[6] >> 4));
    uint32_t raw_z = Mag_ReadRaw18(reg_data[4], reg_data[5], (uint8_t)(reg_data[6] >> 2));
    int32_t signed_x = (int32_t)raw_x - MMC5983MA_RAW_CENTER;
    int32_t signed_y = (int32_t)raw_y - MMC5983MA_RAW_CENTER;
    int32_t signed_z = (int32_t)raw_z - MMC5983MA_RAW_CENTER;

    s_data.mx_uT = (float)signed_x * MMC5983MA_UT_PER_LSB;
    s_data.my_uT = (float)signed_y * MMC5983MA_UT_PER_LSB;
    s_data.mz_uT = (float)signed_z * MMC5983MA_UT_PER_LSB;
    s_data.lastUpdate_ms = now_ms;
    s_data.online = 1U;
}

MagMmc5983maInitResult MagMmc5983ma_Init(void)
{
    uint8_t product_id;

    memset(&s_data, 0, sizeof(s_data));
    s_initialized = 0U;
    s_measure_pending = 0U;

    if (Mag_ReadRegs(MMC5983MA_REG_PRODUCT_ID, &product_id, 1U) != HAL_OK)
    {
        return MagMmc5983ma_InitBusError;
    }

    if (product_id != MMC5983MA_PRODUCT_ID)
    {
        return MagMmc5983ma_InitProductIdError;
    }

    (void)Mag_WriteReg(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST);
    HAL_Delay(10U);
    (void)Mag_WriteReg(MMC5983MA_REG_CTRL1, 0x00U);

    if (Mag_Set() != HAL_OK)
    {
        return MagMmc5983ma_InitBusError;
    }
    HAL_Delay(1U);

    if (Mag_Reset() != HAL_OK)
    {
        return MagMmc5983ma_InitBusError;
    }
    HAL_Delay(1U);

    if (Mag_WriteReg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_AUTO_SR_EN) != HAL_OK)
    {
        return MagMmc5983ma_InitBusError;
    }

    s_initialized = 1U;
    s_data.online = 1U;

    return MagMmc5983ma_InitOk;
}

MagMmc5983maUpdateResult MagMmc5983ma_Update(uint32_t now_ms, SensorsMagData_t *out)
{
    uint8_t status;
    uint8_t reg_data[7];

    if (s_initialized == 0U)
    {
        s_data.online = 0U;
        if (out != NULL)
        {
            *out = s_data;
        }
        return MagMmc5983ma_UpdateBusError;
    }

    if (s_measure_pending == 0U)
    {
        if (Mag_WriteReg(MMC5983MA_REG_CTRL0, (uint8_t)(MMC5983MA_CTRL0_AUTO_SR_EN | MMC5983MA_CTRL0_TM_M)) != HAL_OK)
        {
            s_data.online = 0U;
            s_data.error_count++;
            if (out != NULL)
            {
                *out = s_data;
            }
            return MagMmc5983ma_UpdateBusError;
        }

        s_measure_pending = 1U;
        s_measure_start_ms = now_ms;
        if (out != NULL)
        {
            *out = s_data;
        }
        return MagMmc5983ma_UpdateBusy;
    }

    if (Mag_ReadRegs(MMC5983MA_REG_STATUS, &status, 1U) != HAL_OK)
    {
        s_measure_pending = 0U;
        s_data.online = 0U;
        s_data.error_count++;
        if (out != NULL)
        {
            *out = s_data;
        }
        return MagMmc5983ma_UpdateBusError;
    }

    if ((status & MMC5983MA_STATUS_MEAS_M_DONE) == 0U)
    {
        if ((now_ms - s_measure_start_ms) > MMC5983MA_MEAS_TIMEOUT_MS)
        {
            s_measure_pending = 0U;
            s_data.online = 0U;
            s_data.error_count++;
            if (out != NULL)
            {
                *out = s_data;
            }
            return MagMmc5983ma_UpdateTimeout;
        }

        if (out != NULL)
        {
            *out = s_data;
        }
        return MagMmc5983ma_UpdateBusy;
    }

    if (Mag_ReadRegs(MMC5983MA_REG_XOUT0, reg_data, sizeof(reg_data)) != HAL_OK)
    {
        s_measure_pending = 0U;
        s_data.online = 0U;
        s_data.error_count++;
        if (out != NULL)
        {
            *out = s_data;
        }
        return MagMmc5983ma_UpdateBusError;
    }

    (void)Mag_WriteReg(MMC5983MA_REG_STATUS, MMC5983MA_STATUS_MEAS_M_DONE);
    Mag_ParseRaw(reg_data, now_ms);
    s_measure_pending = 0U;

    if (out != NULL)
    {
        *out = s_data;
    }

    return MagMmc5983ma_UpdateOk;
}

uint8_t MagMmc5983ma_GetData(SensorsMagData_t *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    *out = s_data;
    return s_data.online;
}
