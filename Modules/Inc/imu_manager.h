#ifndef __IMU_MANAGER_H
#define __IMU_MANAGER_H

#include "imu_manager_config.h"
#include <stdint.h>

typedef enum
{
    IMU_CAL_IDLE = 0,
    IMU_CAL_WAIT_STILL,
    IMU_CAL_COLLECTING,
    IMU_CAL_DONE,
    IMU_CAL_FAILED
} ImuCalState_t;

typedef struct
{
    uint8_t imu_id;
    uint8_t online;
    uint8_t healthy;
    uint8_t calibrated;
    uint8_t data_valid;

    uint32_t lastUpdate_ms;
    uint32_t sample_count;

    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    int16_t temp_raw;

    float ax_meas_mps2;
    float ay_meas_mps2;
    float az_meas_mps2;
    float gx_meas_radps;
    float gy_meas_radps;
    float gz_meas_radps;

    float acc_bias_mps2[3];
    float gyro_bias_radps[3];

    float ax_cal_mps2;
    float ay_cal_mps2;
    float az_cal_mps2;
    float gx_cal_radps;
    float gy_cal_radps;
    float gz_cal_radps;

    float temperature_degC;
} SensorsImuData_t;

void ImuManager_Init(void);
void ImuManager_Update(void);
uint8_t ImuManager_GetImu(uint8_t imu_id, SensorsImuData_t *out);
uint8_t ImuManager_GetPrimaryImu(SensorsImuData_t *out);
uint8_t ImuManager_IsOnline(uint8_t imu_id);
uint8_t ImuManager_IsPrimaryOnline(void);
uint8_t ImuManager_GetPrimaryId(void);
void ImuManager_SetPrimaryId(uint8_t imu_id);
void ImuManager_StartStaticCalibration(void);
void ImuManager_CancelStaticCalibration(void);
uint8_t ImuManager_IsCalibrating(void);
uint8_t ImuManager_IsCalibrated(uint8_t imu_id);
ImuCalState_t ImuManager_GetCalibrationState(void);

#endif /* __IMU_MANAGER_H */
