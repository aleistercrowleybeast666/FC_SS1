#ifndef __IMU_ICM42688P_H
#define __IMU_ICM42688P_H

#include "imu_manager.h"
#include <stdint.h>

int ImuIcm42688p_Init(void);
int ImuIcm42688p_Read(SensorsImuData_t *out);
uint8_t ImuIcm42688p_IsOnline(void);

#endif /* __IMU_ICM42688P_H */
