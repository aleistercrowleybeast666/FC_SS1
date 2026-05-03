#ifndef __SENSOR_CONFIG_H
#define __SENSOR_CONFIG_H

#define SENSOR_TASK_PERIOD_MS       10U

#define GNSS_RX_DMA_BUF_SIZE        128U
#define GNSS_RX_RINGBUF_SIZE        1024U
#define GNSS_TIMEOUT_MS             1500U
#define GNSS_DEFAULT_BAUDRATE       38400U
#define GNSS_TARGET_BAUDRATE        921600U
#define GNSS_MAX_BAUDRATE           921600U
#define GNSS_DEFAULT_RATE_HZ        1U
#define GNSS_TARGET_RATE_HZ         25U
#define GNSS_MAX_RATE_HZ            25U
#define GNSS_CFG_TIMEOUT_MS         100U
#define GNSS_SIGNAL_RESET_WAIT_MS   600U
#define GNSS_CFG_LAYER_RAM          0x01U
#define GNSS_CFG_LAYER_BBR          0x02U
#define GNSS_CFG_LAYER_FLASH        0x04U
#define GNSS_CFG_LAYER_RAM_FLASH    (GNSS_CFG_LAYER_RAM | GNSS_CFG_LAYER_FLASH)
#define GNSS_CFG_LAYER_ALL          (GNSS_CFG_LAYER_RAM | GNSS_CFG_LAYER_BBR | GNSS_CFG_LAYER_FLASH)
#define GNSS_DYNMODEL_PORTABLE      0U
#define GNSS_DYNMODEL_STATIONARY    2U
#define GNSS_DYNMODEL_PEDESTRIAN    3U
#define GNSS_DYNMODEL_AUTOMOTIVE    4U
#define GNSS_DYNMODEL_SEA           5U
#define GNSS_DYNMODEL_AIRBORNE_1G   6U
#define GNSS_DYNMODEL_AIRBORNE_2G   7U
#define GNSS_DYNMODEL_AIRBORNE_4G   8U
#define GNSS_TARGET_DYNMODEL        GNSS_DYNMODEL_AIRBORNE_4G
#define GNSS_NAV_MIN_SV             8U
#define GNSS_NAV_MAX_HACC_MM        1500U
#define GNSS_NAV_MAX_VACC_MM        2500U
#define GNSS_NAV_MAX_AGE_MS         500U
#define GNSS_RM_M                   6335439.0f
#define GNSS_RN_M                   6378137.0f
#define GNSS_LOCAL_AUTO_ORIGIN_ENABLE 1U
#define GNSS_DEG_TO_RAD             0.017453292519943295f
/*
 * GNSS 天线相位中心相对飞控/IMU 参考点的位置，单位 m，机体系：
 * X 机体前向为正，Y 机体左向为正，Z 机体上向为正。
 * 第一版不参与 GNSS 经纬度转 XY。
 * 后续如果 ESKF 融合 GNSS 位置/速度，可在观测模型中使用：
 * p_gnss = p_body + R_nb * r_gnss_body
 * v_gnss = v_body + R_nb * (omega_body x r_gnss_body)
 */
#define GNSS_LEVER_ARM_X_M          0.050f
#define GNSS_LEVER_ARM_Y_M          0.000f
#define GNSS_LEVER_ARM_Z_M          0.000f

#define BARO_TIMEOUT_MS             200U
#define MAG_TIMEOUT_MS              200U

#define BMP390_I2C_ADDR             0x76U
#define MMC5983MA_I2C_ADDR          0x30U
#define BARO_SEA_LEVEL_PRESSURE_PA  101325.0f
#define SENSOR_I2C_TIMEOUT_MS       10U

#endif /* __SENSOR_CONFIG_H */
