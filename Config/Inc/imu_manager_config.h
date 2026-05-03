#ifndef __IMU_MANAGER_CONFIG_H
#define __IMU_MANAGER_CONFIG_H

#define IMU_COUNT                   2U
#define IMU_ID_BMI088               0U
#define IMU_ID_ICM42688P            1U
#define IMU_PRIMARY_DEFAULT_ID      IMU_ID_BMI088
/*
 * 两个 IMU 相对飞控/机体参考点的杆臂位置，单位 m，机体系：
 * X 机体前向为正，Y 机体左向为正，Z 机体上向为正。
 * 当前先使用占位值，后续做双 IMU 安装误差建模或更精细的一致性分析时可再实测修正。
 */
#define IMU_BMI088_LEVER_ARM_X_M    0.000f
#define IMU_BMI088_LEVER_ARM_Y_M    0.000f
#define IMU_BMI088_LEVER_ARM_Z_M    0.000f
#define IMU_ICM42688P_LEVER_ARM_X_M 0.000f
#define IMU_ICM42688P_LEVER_ARM_Y_M 0.000f
#define IMU_ICM42688P_LEVER_ARM_Z_M 0.000f

#define IMU_STATIC_CAL_WAIT_MS      1000U
#define IMU_STATIC_CAL_DURATION_MS  2000U
#define IMU_GRAVITY_MPS2            9.80665f
/*
 * 静止直立放置时期望加速度，按当前机体系定义填写。
 * 当前先占位为 Z 轴 +g。如果实测静止时 az 为 -g，再改成 -9.80665f。
 */
#define IMU_STATIC_ACC_REF_X_MPS2   0.0f
#define IMU_STATIC_ACC_REF_Y_MPS2   0.0f
#define IMU_STATIC_ACC_REF_Z_MPS2   9.80665f

#define IMU_TIMEOUT_MS              50U
#define IMU_ACCEL_RANGE_G           24U
#define IMU_GYRO_RANGE_DPS          2000U
#define IMU_TARGET_ODR_HZ           200U
#define IMU_GYRO_DIFF_LIMIT_RADPS   1.0f
#define IMU_ACC_DIFF_LIMIT_MPS2     5.0f
#define IMU_SPI_TIMEOUT_MS          10U
#define IMU_REINIT_INTERVAL_MS      1000U

#endif /* __IMU_MANAGER_CONFIG_H */
