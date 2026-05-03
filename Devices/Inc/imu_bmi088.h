#ifndef __IMU_BMI088_H
#define __IMU_BMI088_H

#include "imu_manager.h"
#include <stdint.h>

int ImuBmi088_Init(void);
int ImuBmi088_Read(SensorsImuData_t *out);
uint8_t ImuBmi088_IsOnline(void);

#endif /* __IMU_BMI088_H */
