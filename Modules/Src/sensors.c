#include "sensors.h"

#include "baro_bmp390.h"
#include "cmsis_gcc.h"
#include "gnss_neo_m9n.h"
#include "imu_manager.h"
#include "mag_mmc5983ma.h"
#include "main.h"
#include "sensor_config.h"
#include <math.h>
#include <string.h>

#define SENSORS_BARO_PERIOD_MS      20U
#define SENSORS_MAG_PERIOD_MS       20U
#define SENSORS_LATLON_TO_DEG       0.0000001f

/*
 * SensorsGnssData_t 字段说明：
 * iTOW：GNSS 周内时间，单位 ms，来自 UBX-NAV-PVT。
 * lastUpdate_ms：最近一次收到合法 UBX-NAV-PVT 的系统 tick，单位 ms。
 * online：GNSS 在线标志，最近 GNSS_TIMEOUT_MS 内收到合法 UBX-NAV-PVT 时为 1。
 * fixType：定位类型，来自 UBX-NAV-PVT，2 表示 2D fix，3 表示 3D fix。
 * gnssFixOK：GNSS fix 有效标志，来自 UBX-NAV-PVT flags bit0。
 * numSV：参与解算的卫星数量。
 * hasValidFix：已定位标志，online && fixType >= 2 && gnssFixOK == 1。
 * usableForNav：导航/EKF 可用标志，比 hasValidFix 更严格，包含 3D fix、卫星数、精度和数据年龄判断。
 * lon：经度，单位 1e-7 deg，东经为正。
 * lat：纬度，单位 1e-7 deg，北纬为正。
 * height：椭球高，单位 mm。
 * hMSL：海平面高度，单位 mm。
 * hAcc：水平位置精度估计，单位 mm。
 * vAcc：垂直位置精度估计，单位 mm。
 * velN：北向速度，单位 mm/s，向北为正。
 * velE：东向速度，单位 mm/s，向东为正。
 * velD：地向速度，单位 mm/s，向下为正。
 * gSpeed：地面速度，单位 mm/s。
 * headMot：运动航向，单位 1e-5 deg。
 * sAcc：速度精度估计，单位 mm/s。
 * headAcc：航向精度估计，单位 1e-5 deg。
 * local_x_m：本地坐标 X，单位 m，向东为正。
 * local_y_m：本地坐标 Y，单位 m，向北为正。
 * local_xy_valid：本地 XY 有效标志，有原点且当前 GNSS 有效定位时为 1。
 * origin_lat：本地坐标原点纬度，单位 1e-7 deg。
 * origin_lon：本地坐标原点经度，单位 1e-7 deg。
 */
static SensorsGnssData_t s_gnss_snapshot;
static SensorsBaroData_t s_baro_snapshot;
static SensorsMagData_t s_mag_snapshot;
static uint32_t s_baro_last_ms = 0U;
static uint32_t s_mag_last_ms = 0U;
static uint8_t s_initialized = 0U;
static uint8_t s_gnss_origin_valid = 0U;
static int32_t s_gnss_origin_lat = 0;
static int32_t s_gnss_origin_lon = 0;

static uint32_t Sensors_IrqLock(void);
static void Sensors_IrqUnlock(uint32_t primask);
static void Sensors_GnssSnapshotUpdate(const SensorsGnssData_t *data);
static void Sensors_BaroSnapshotUpdate(const SensorsBaroData_t *data);
static void Sensors_MagSnapshotUpdate(const SensorsMagData_t *data);
static uint8_t Sensors_GnssOriginGet(int32_t *lat_1e7, int32_t *lon_1e7);
static void Sensors_GnssLocalUpdate(SensorsGnssData_t *data);

static uint32_t Sensors_IrqLock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Sensors_IrqUnlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void Sensors_GnssSnapshotUpdate(const SensorsGnssData_t *data)
{
    uint32_t primask;

    if (data == NULL)
    {
        return;
    }

    primask = Sensors_IrqLock();
    s_gnss_snapshot = *data;
    Sensors_IrqUnlock(primask);
}

static void Sensors_BaroSnapshotUpdate(const SensorsBaroData_t *data)
{
    uint32_t primask;

    if (data == NULL)
    {
        return;
    }

    primask = Sensors_IrqLock();
    s_baro_snapshot = *data;
    Sensors_IrqUnlock(primask);
}

static void Sensors_MagSnapshotUpdate(const SensorsMagData_t *data)
{
    uint32_t primask;

    if (data == NULL)
    {
        return;
    }

    primask = Sensors_IrqLock();
    s_mag_snapshot = *data;
    Sensors_IrqUnlock(primask);
}

static uint8_t Sensors_GnssOriginGet(int32_t *lat_1e7, int32_t *lon_1e7)
{
    uint8_t valid;
    uint32_t primask;

    primask = Sensors_IrqLock();
    valid = s_gnss_origin_valid;
    if (lat_1e7 != NULL)
    {
        *lat_1e7 = s_gnss_origin_lat;
    }
    if (lon_1e7 != NULL)
    {
        *lon_1e7 = s_gnss_origin_lon;
    }
    Sensors_IrqUnlock(primask);

    return valid;
}

static void Sensors_GnssLocalUpdate(SensorsGnssData_t *data)
{
    int32_t origin_lat;
    int32_t origin_lon;
    float d_lat_rad;
    float d_lon_rad;
    float origin_lat_rad;

    if (data == NULL)
    {
        return;
    }

    data->local_x_m = 0.0f;
    data->local_y_m = 0.0f;
    data->local_xy_valid = 0U;
    data->origin_lat = 0;
    data->origin_lon = 0;

    if (Sensors_GnssOriginGet(&origin_lat, &origin_lon) == 0U)
    {
        return;
    }

    data->origin_lat = origin_lat;
    data->origin_lon = origin_lon;

    if (data->hasValidFix == 0U)
    {
        return;
    }

    d_lat_rad = ((float)(data->lat - origin_lat)) * SENSORS_LATLON_TO_DEG * GNSS_DEG_TO_RAD;
    d_lon_rad = ((float)(data->lon - origin_lon)) * SENSORS_LATLON_TO_DEG * GNSS_DEG_TO_RAD;
    origin_lat_rad = ((float)origin_lat) * SENSORS_LATLON_TO_DEG * GNSS_DEG_TO_RAD;

    data->local_x_m = d_lon_rad * GNSS_RN_M * cosf(origin_lat_rad);
    data->local_y_m = d_lat_rad * GNSS_RM_M;
    data->local_xy_valid = 1U;
}

void Sensors_Init(void)
{
    uint32_t now_ms = HAL_GetTick();
    SensorsGnssData_t gnss_data;
    SensorsBaroData_t baro_data;
    SensorsMagData_t mag_data;

    if (s_initialized != 0U)
    {
        return;
    }

    memset(&s_gnss_snapshot, 0, sizeof(s_gnss_snapshot));
    memset(&s_baro_snapshot, 0, sizeof(s_baro_snapshot));
    memset(&s_mag_snapshot, 0, sizeof(s_mag_snapshot));

    (void)GnssNeoM9n_Init();
    (void)BaroBmp390_Init();
    (void)MagMmc5983ma_Init();

    (void)GnssNeoM9n_GetData(&gnss_data);
    (void)BaroBmp390_GetData(&baro_data);
    (void)MagMmc5983ma_GetData(&mag_data);
    Sensors_GnssLocalUpdate(&gnss_data);
    Sensors_GnssSnapshotUpdate(&gnss_data);
    Sensors_BaroSnapshotUpdate(&baro_data);
    Sensors_MagSnapshotUpdate(&mag_data);

    s_baro_last_ms = now_ms;
    s_mag_last_ms = now_ms;
    s_initialized = 1U;
}

void Sensors_TaskUpdate(void)
{
    uint32_t now_ms = HAL_GetTick();
    SensorsGnssData_t gnss_data;
    SensorsBaroData_t baro_data;
    SensorsMagData_t mag_data;

    if (s_initialized == 0U)
    {
        return;
    }

    (void)GnssNeoM9n_Process(now_ms);
    (void)GnssNeoM9n_GetData(&gnss_data);
#if (GNSS_LOCAL_AUTO_ORIGIN_ENABLE != 0U)
    if ((Sensors_GnssOriginGet(NULL, NULL) == 0U) && (gnss_data.hasValidFix != 0U))
    {
        Sensors_GnssSetOrigin(gnss_data.lat, gnss_data.lon);
    }
#endif
    Sensors_GnssLocalUpdate(&gnss_data);
    Sensors_GnssSnapshotUpdate(&gnss_data);
    ImuManager_Update();

    if ((now_ms - s_baro_last_ms) >= SENSORS_BARO_PERIOD_MS)
    {
        s_baro_last_ms = now_ms;
        (void)BaroBmp390_Update(now_ms, &baro_data);
        Sensors_BaroSnapshotUpdate(&baro_data);
    }

    if ((now_ms - s_mag_last_ms) >= SENSORS_MAG_PERIOD_MS)
    {
        s_mag_last_ms = now_ms;
        (void)MagMmc5983ma_Update(now_ms, &mag_data);
        Sensors_MagSnapshotUpdate(&mag_data);
    }
}

uint8_t Sensors_GetGnss(SensorsGnssData_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return 0U;
    }

    primask = Sensors_IrqLock();
    *out = s_gnss_snapshot;
    Sensors_IrqUnlock(primask);

    return out->online;
}

uint8_t Sensors_GetBaro(SensorsBaroData_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return 0U;
    }

    primask = Sensors_IrqLock();
    *out = s_baro_snapshot;
    Sensors_IrqUnlock(primask);

    return out->online;
}

uint8_t Sensors_GetMag(SensorsMagData_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return 0U;
    }

    primask = Sensors_IrqLock();
    *out = s_mag_snapshot;
    Sensors_IrqUnlock(primask);

    return out->online;
}

uint8_t Sensors_GnssIsOnline(void)
{
    SensorsGnssData_t gnss_data;

    return Sensors_GetGnss(&gnss_data);
}

uint8_t Sensors_GnssHasValidFix(void)
{
    SensorsGnssData_t gnss_data;

    (void)Sensors_GetGnss(&gnss_data);
    return gnss_data.hasValidFix;
}

uint8_t Sensors_GnssIsUsableForNav(void)
{
    SensorsGnssData_t gnss_data;

    (void)Sensors_GetGnss(&gnss_data);
    return gnss_data.usableForNav;
}

uint8_t Sensors_BaroIsOnline(void)
{
    SensorsBaroData_t baro_data;
    uint32_t now_ms = HAL_GetTick();

    (void)Sensors_GetBaro(&baro_data);

    return ((baro_data.online != 0U) &&
            (baro_data.lastUpdate_ms != 0U) &&
            ((now_ms - baro_data.lastUpdate_ms) <= BARO_TIMEOUT_MS)) ? 1U : 0U;
}

uint8_t Sensors_MagIsOnline(void)
{
    SensorsMagData_t mag_data;
    uint32_t now_ms = HAL_GetTick();

    (void)Sensors_GetMag(&mag_data);

    return ((mag_data.online != 0U) &&
            (mag_data.lastUpdate_ms != 0U) &&
            ((now_ms - mag_data.lastUpdate_ms) <= MAG_TIMEOUT_MS)) ? 1U : 0U;
}

uint8_t Sensors_GnssSetOriginCurrent(void)
{
    SensorsGnssData_t gnss_data;

    (void)Sensors_GetGnss(&gnss_data);
    if (gnss_data.hasValidFix == 0U)
    {
        return 0U;
    }

    Sensors_GnssSetOrigin(gnss_data.lat, gnss_data.lon);
    return 1U;
}

void Sensors_GnssSetOrigin(int32_t lat_1e7, int32_t lon_1e7)
{
    SensorsGnssData_t gnss_data;
    uint32_t primask;

    primask = Sensors_IrqLock();
    s_gnss_origin_lat = lat_1e7;
    s_gnss_origin_lon = lon_1e7;
    s_gnss_origin_valid = 1U;
    gnss_data = s_gnss_snapshot;
    Sensors_IrqUnlock(primask);

    Sensors_GnssLocalUpdate(&gnss_data);
    Sensors_GnssSnapshotUpdate(&gnss_data);
}

void Sensors_GnssClearOrigin(void)
{
    uint32_t primask;

    primask = Sensors_IrqLock();
    s_gnss_origin_lat = 0;
    s_gnss_origin_lon = 0;
    s_gnss_origin_valid = 0U;
    s_gnss_snapshot.local_x_m = 0.0f;
    s_gnss_snapshot.local_y_m = 0.0f;
    s_gnss_snapshot.local_xy_valid = 0U;
    s_gnss_snapshot.origin_lat = 0;
    s_gnss_snapshot.origin_lon = 0;
    Sensors_IrqUnlock(primask);
}

uint8_t Sensors_GnssHasLocalOrigin(void)
{
    return Sensors_GnssOriginGet(NULL, NULL);
}
