#ifndef __SENSORS_H
#define __SENSORS_H

#include <stdint.h>

typedef struct
{
    uint32_t iTOW;
    uint32_t lastUpdate_ms;
    uint8_t online;
    uint8_t fixType;
    uint8_t gnssFixOK;
    uint8_t numSV;
    uint8_t hasValidFix;
    uint8_t usableForNav;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    float local_x_m;
    float local_y_m;
    uint8_t local_xy_valid;
    int32_t origin_lat;
    int32_t origin_lon;
} SensorsGnssData_t;

typedef struct
{
    float pressure_pa;
    float temperature_degC;
    float altitude_m;
    uint32_t lastUpdate_ms;
    uint8_t online;
    uint32_t error_count;
} SensorsBaroData_t;

typedef struct
{
    float mx_uT;
    float my_uT;
    float mz_uT;
    uint32_t lastUpdate_ms;
    uint8_t online;
    uint32_t error_count;
} SensorsMagData_t;

typedef struct SensorsImuRawData_t
{
    uint8_t imu_id;
    uint32_t lastUpdate_ms;
    uint8_t online;
} SensorsImuRawData_t;

typedef struct SensorsImuEkfData_t
{
    uint32_t lastUpdate_ms;
    uint8_t valid;
} SensorsImuEkfData_t;

void Sensors_Init(void);
void Sensors_TaskUpdate(void);
uint8_t Sensors_GetGnss(SensorsGnssData_t *out);
uint8_t Sensors_GetBaro(SensorsBaroData_t *out);
uint8_t Sensors_GetMag(SensorsMagData_t *out);

/* GNSS module online: a valid UBX-NAV-PVT was received within GNSS_TIMEOUT_MS.
 * This can be used as a GNSS ping/alive indication. */
//GNSS是否在线
uint8_t Sensors_GnssIsOnline(void);

/* GNSS has a valid fix: online && fixType >= 2 && gnssFixOK == 1.
 * This means the receiver has found satellites and produced a usable position fix. */
//GNSS是否寻星成功
uint8_t Sensors_GnssHasValidFix(void);

/* GNSS usable for navigation/EKF: stricter than HasValidFix, requiring 3D fix,
 * enough satellites, hAcc/vAcc within limits, and fresh data. */
//GNSS是否可用于导航
uint8_t Sensors_GnssIsUsableForNav(void);
//气压计是否在线
uint8_t Sensors_BaroIsOnline(void);
//磁力计是否在线
uint8_t Sensors_MagIsOnline(void);

/* 使用当前 GNSS 经纬度设置本地 XY 原点。
 * 仅当 GNSS 已有有效定位时成功，成功返回 1，否则返回 0。 */
uint8_t Sensors_GnssSetOriginCurrent(void);
/* 手动设置本地 XY 原点。
 * lat_1e7/lon_1e7 单位为 1e-7 deg，设置后 Sensors_GetGnss() 会带出 origin_lat/origin_lon。 */
void Sensors_GnssSetOrigin(int32_t lat_1e7, int32_t lon_1e7);
/* 清除本地 XY 原点。
 * 清除后 local_xy_valid 为 0，local_x_m/local_y_m 归零。 */
void Sensors_GnssClearOrigin(void);
/* 查询当前是否已经设置本地 XY 原点。
 * 已设置返回 1，未设置返回 0；该接口只读取快照/状态，不访问 GNSS 或 I2C。 */
uint8_t Sensors_GnssHasLocalOrigin(void);

uint8_t Sensors_GetImuRaw(uint8_t imu_id, SensorsImuRawData_t *out);
uint8_t Sensors_GetImuForEkf(SensorsImuEkfData_t *out);

#endif /* __SENSORS_H */
