#include "baro_bmp390.h"

#include "i2c.h"
#include "sensor_config.h"
#include <math.h>
#include <string.h>

#define BMP390_REG_CHIP_ID          0x00U
#define BMP390_REG_DATA             0x04U
#define BMP390_REG_PWR_CTRL         0x1BU
#define BMP390_REG_OSR              0x1CU
#define BMP390_REG_ODR              0x1DU
#define BMP390_REG_CONFIG           0x1FU
#define BMP390_REG_CALIB            0x31U
#define BMP390_CHIP_ID              0x60U
#define BMP390_CALIB_LEN            21U
#define BMP390_I2C_DEV_ADDR         (BMP390_I2C_ADDR << 1)

typedef struct
{
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
} BaroBmp390Calib_t;

static BaroBmp390Calib_t s_calib;
static SensorsBaroData_t s_data;
static uint8_t s_initialized = 0U;

static HAL_StatusTypeDef Baro_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef Baro_WriteReg(uint8_t reg, uint8_t data);
static uint16_t Baro_ReadU16Le(const uint8_t *data);
static int16_t Baro_ReadI16Le(const uint8_t *data);
static void Baro_ParseCalib(const uint8_t *reg_data);
static double Baro_Pow(double base, uint8_t power);
static double Baro_CompensateTemperature(uint32_t raw_temp);
static double Baro_CompensatePressure(uint32_t raw_press);

static HAL_StatusTypeDef Baro_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            BMP390_I2C_DEV_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            len,
                            SENSOR_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Baro_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c2,
                             BMP390_I2C_DEV_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1U,
                             SENSOR_I2C_TIMEOUT_MS);
}

static uint16_t Baro_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static int16_t Baro_ReadI16Le(const uint8_t *data)
{
    return (int16_t)Baro_ReadU16Le(data);
}

static void Baro_ParseCalib(const uint8_t *reg_data)
{
    uint16_t par_t1 = Baro_ReadU16Le(&reg_data[0]);
    uint16_t par_t2 = Baro_ReadU16Le(&reg_data[2]);
    int8_t par_t3 = (int8_t)reg_data[4];
    int16_t par_p1 = Baro_ReadI16Le(&reg_data[5]);
    int16_t par_p2 = Baro_ReadI16Le(&reg_data[7]);
    int8_t par_p3 = (int8_t)reg_data[9];
    int8_t par_p4 = (int8_t)reg_data[10];
    uint16_t par_p5 = Baro_ReadU16Le(&reg_data[11]);
    uint16_t par_p6 = Baro_ReadU16Le(&reg_data[13]);
    int8_t par_p7 = (int8_t)reg_data[15];
    int8_t par_p8 = (int8_t)reg_data[16];
    int16_t par_p9 = Baro_ReadI16Le(&reg_data[17]);
    int8_t par_p10 = (int8_t)reg_data[19];
    int8_t par_p11 = (int8_t)reg_data[20];

    s_calib.par_t1 = (double)par_t1 * 256.0;
    s_calib.par_t2 = (double)par_t2 / 1073741824.0;
    s_calib.par_t3 = (double)par_t3 / 281474976710656.0;
    s_calib.par_p1 = (double)(par_p1 - 16384) / 1048576.0;
    s_calib.par_p2 = (double)(par_p2 - 16384) / 536870912.0;
    s_calib.par_p3 = (double)par_p3 / 4294967296.0;
    s_calib.par_p4 = (double)par_p4 / 137438953472.0;
    s_calib.par_p5 = (double)par_p5 * 8.0;
    s_calib.par_p6 = (double)par_p6 / 64.0;
    s_calib.par_p7 = (double)par_p7 / 256.0;
    s_calib.par_p8 = (double)par_p8 / 32768.0;
    s_calib.par_p9 = (double)par_p9 / 281474976710656.0;
    s_calib.par_p10 = (double)par_p10 / 281474976710656.0;
    s_calib.par_p11 = (double)par_p11 / 36893488147419103232.0;
}

static double Baro_Pow(double base, uint8_t power)
{
    double value = 1.0;

    while (power != 0U)
    {
        value *= base;
        power--;
    }

    return value;
}

static double Baro_CompensateTemperature(uint32_t raw_temp)
{
    double partial_data1 = (double)raw_temp - s_calib.par_t1;
    double partial_data2 = partial_data1 * s_calib.par_t2;

    s_calib.t_lin = partial_data2 + (partial_data1 * partial_data1) * s_calib.par_t3;
    return s_calib.t_lin;
}

static double Baro_CompensatePressure(uint32_t raw_press)
{
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;
    double raw_press_d = (double)raw_press;

    partial_data1 = s_calib.par_p6 * s_calib.t_lin;
    partial_data2 = s_calib.par_p7 * Baro_Pow(s_calib.t_lin, 2U);
    partial_data3 = s_calib.par_p8 * Baro_Pow(s_calib.t_lin, 3U);
    partial_out1 = s_calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = s_calib.par_p2 * s_calib.t_lin;
    partial_data2 = s_calib.par_p3 * Baro_Pow(s_calib.t_lin, 2U);
    partial_data3 = s_calib.par_p4 * Baro_Pow(s_calib.t_lin, 3U);
    partial_out2 = raw_press_d * (s_calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = Baro_Pow(raw_press_d, 2U);
    partial_data2 = s_calib.par_p9 + s_calib.par_p10 * s_calib.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + Baro_Pow(raw_press_d, 3U) * s_calib.par_p11;

    return partial_out1 + partial_out2 + partial_data4;
}

BaroBmp390InitResult BaroBmp390_Init(void)
{
    uint8_t chip_id;
    uint8_t calib[BMP390_CALIB_LEN];

    memset(&s_calib, 0, sizeof(s_calib));
    memset(&s_data, 0, sizeof(s_data));
    s_initialized = 0U;

    if (Baro_ReadRegs(BMP390_REG_CHIP_ID, &chip_id, 1U) != HAL_OK)
    {
        return BaroBmp390_InitBusError;
    }

    if (chip_id != BMP390_CHIP_ID)
    {
        return BaroBmp390_InitChipIdError;
    }

    if (Baro_ReadRegs(BMP390_REG_CALIB, calib, BMP390_CALIB_LEN) != HAL_OK)
    {
        return BaroBmp390_InitCalibError;
    }

    Baro_ParseCalib(calib);

    if ((Baro_WriteReg(BMP390_REG_OSR, 0x00U) != HAL_OK) ||
        (Baro_WriteReg(BMP390_REG_ODR, 0x03U) != HAL_OK) ||
        (Baro_WriteReg(BMP390_REG_CONFIG, 0x04U) != HAL_OK) ||
        (Baro_WriteReg(BMP390_REG_PWR_CTRL, 0x33U) != HAL_OK))
    {
        return BaroBmp390_InitBusError;
    }

    s_initialized = 1U;
    s_data.online = 1U;

    return BaroBmp390_InitOk;
}

BaroBmp390UpdateResult BaroBmp390_Update(uint32_t now_ms, SensorsBaroData_t *out)
{
    uint8_t reg_data[6];
    uint32_t raw_press;
    uint32_t raw_temp;
    double temperature;
    double pressure;

    if (s_initialized == 0U)
    {
        s_data.online = 0U;
        if (out != NULL)
        {
            *out = s_data;
        }
        return BaroBmp390_UpdateNotReady;
    }

    if (Baro_ReadRegs(BMP390_REG_DATA, reg_data, sizeof(reg_data)) != HAL_OK)
    {
        s_data.online = 0U;
        s_data.error_count++;
        if (out != NULL)
        {
            *out = s_data;
        }
        return BaroBmp390_UpdateBusError;
    }

    raw_press = ((uint32_t)reg_data[0]) |
                ((uint32_t)reg_data[1] << 8) |
                ((uint32_t)reg_data[2] << 16);
    raw_temp = ((uint32_t)reg_data[3]) |
               ((uint32_t)reg_data[4] << 8) |
               ((uint32_t)reg_data[5] << 16);

    temperature = Baro_CompensateTemperature(raw_temp);
    pressure = Baro_CompensatePressure(raw_press);

    s_data.pressure_pa = (float)pressure;
    s_data.temperature_degC = (float)temperature;
    s_data.altitude_m = 44330.0f *
                        (1.0f - powf((float)(pressure / (double)BARO_SEA_LEVEL_PRESSURE_PA), 0.19029495f));
    s_data.lastUpdate_ms = now_ms;
    s_data.online = 1U;

    if (out != NULL)
    {
        *out = s_data;
    }

    return BaroBmp390_UpdateOk;
}

uint8_t BaroBmp390_GetData(SensorsBaroData_t *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    *out = s_data;
    return s_data.online;
}
