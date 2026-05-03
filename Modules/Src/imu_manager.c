#include "imu_manager.h"

#include "cmsis_gcc.h"
#include "imu_bmi088.h"
#include "imu_icm42688p.h"
#include "imu_manager_config.h"
#include "main.h"
#include <math.h>
#include <string.h>

#define IMU_STATIC_CAL_MIN_SAMPLES  10U

typedef struct
{
    double acc_sum[3];
    double gyro_sum[3];
    uint32_t sample_count;
} ImuCalAccumulator_t;

static SensorsImuData_t s_imu[IMU_COUNT];
static uint8_t s_primary_id = IMU_PRIMARY_DEFAULT_ID;
static uint8_t s_initialized = 0U;
static ImuCalState_t s_cal_state = IMU_CAL_IDLE;
static uint32_t s_cal_state_start_ms = 0U;
static ImuCalAccumulator_t s_cal_accum[IMU_COUNT];
static uint32_t s_consistency_error_count[IMU_COUNT];
static uint32_t s_last_reinit_try_ms[IMU_COUNT];

static uint32_t ImuManager_IrqLock(void);
static void ImuManager_IrqUnlock(uint32_t primask);
static void ImuManager_DataApplyBias(SensorsImuData_t *data);
static uint8_t ImuManager_DataFinite(const SensorsImuData_t *data);
static uint8_t ImuManager_DataHealthCheck(const SensorsImuData_t *data, uint32_t now_ms);
static void ImuManager_SnapshotSet(uint8_t imu_id, const SensorsImuData_t *data);
static void ImuManager_UpdateOne(uint8_t imu_id, int (*read_fn)(SensorsImuData_t *));
static void ImuManager_UpdateHealth(uint32_t now_ms);
static void ImuManager_UpdatePrimary(void);
static void ImuManager_CheckConsistency(void);
static void ImuManager_CalResetAccum(void);
static void ImuManager_CalCollect(void);
static void ImuManager_CalFinish(void);
static void ImuManager_CalUpdate(uint32_t now_ms);
static void ImuManager_ReinitOfflineImu(uint32_t now_ms);

static uint32_t ImuManager_IrqLock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void ImuManager_IrqUnlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void ImuManager_DataApplyBias(SensorsImuData_t *data)
{
    if (data == NULL)
    {
        return;
    }

    data->ax_cal_mps2 = data->ax_meas_mps2 - data->acc_bias_mps2[0];
    data->ay_cal_mps2 = data->ay_meas_mps2 - data->acc_bias_mps2[1];
    data->az_cal_mps2 = data->az_meas_mps2 - data->acc_bias_mps2[2];
    data->gx_cal_radps = data->gx_meas_radps - data->gyro_bias_radps[0];
    data->gy_cal_radps = data->gy_meas_radps - data->gyro_bias_radps[1];
    data->gz_cal_radps = data->gz_meas_radps - data->gyro_bias_radps[2];
}

static uint8_t ImuManager_DataFinite(const SensorsImuData_t *data)
{
    if (data == NULL)
    {
        return 0U;
    }

    return (isfinite(data->ax_meas_mps2) &&
            isfinite(data->ay_meas_mps2) &&
            isfinite(data->az_meas_mps2) &&
            isfinite(data->gx_meas_radps) &&
            isfinite(data->gy_meas_radps) &&
            isfinite(data->gz_meas_radps) &&
            isfinite(data->ax_cal_mps2) &&
            isfinite(data->ay_cal_mps2) &&
            isfinite(data->az_cal_mps2) &&
            isfinite(data->gx_cal_radps) &&
            isfinite(data->gy_cal_radps) &&
            isfinite(data->gz_cal_radps)) ? 1U : 0U;
}

static uint8_t ImuManager_DataHealthCheck(const SensorsImuData_t *data, uint32_t now_ms)
{
    float acc_norm;

    if ((data == NULL) ||
        (data->online == 0U) ||
        (data->data_valid == 0U) ||
        (data->lastUpdate_ms == 0U) ||
        ((now_ms - data->lastUpdate_ms) > IMU_TIMEOUT_MS) ||
        (ImuManager_DataFinite(data) == 0U))
    {
        return 0U;
    }

    acc_norm = sqrtf((data->ax_cal_mps2 * data->ax_cal_mps2) +
                     (data->ay_cal_mps2 * data->ay_cal_mps2) +
                     (data->az_cal_mps2 * data->az_cal_mps2));

    return ((acc_norm >= (0.5f * IMU_GRAVITY_MPS2)) &&
            (acc_norm <= (2.0f * IMU_GRAVITY_MPS2))) ? 1U : 0U;
}

static void ImuManager_SnapshotSet(uint8_t imu_id, const SensorsImuData_t *data)
{
    uint32_t primask;

    if ((imu_id >= IMU_COUNT) || (data == NULL))
    {
        return;
    }

    primask = ImuManager_IrqLock();
    s_imu[imu_id] = *data;
    ImuManager_IrqUnlock(primask);
}

static void ImuManager_UpdateOne(uint8_t imu_id, int (*read_fn)(SensorsImuData_t *))
{
    SensorsImuData_t data;
    SensorsImuData_t old_data;
    uint32_t primask;

    if ((imu_id >= IMU_COUNT) || (read_fn == NULL))
    {
        return;
    }

    primask = ImuManager_IrqLock();
    old_data = s_imu[imu_id];
    ImuManager_IrqUnlock(primask);

    if (read_fn(&data) == 0)
    {
        data.imu_id = imu_id;
        data.sample_count = old_data.sample_count + 1U;
        data.calibrated = old_data.calibrated;
        memcpy(data.acc_bias_mps2, old_data.acc_bias_mps2, sizeof(data.acc_bias_mps2));
        memcpy(data.gyro_bias_radps, old_data.gyro_bias_radps, sizeof(data.gyro_bias_radps));
        ImuManager_DataApplyBias(&data);
        ImuManager_SnapshotSet(imu_id, &data);
    }
}

static void ImuManager_UpdateHealth(uint32_t now_ms)
{
    uint8_t i;
    uint32_t primask;

    primask = ImuManager_IrqLock();
    for (i = 0U; i < IMU_COUNT; i++)
    {
        s_imu[i].online = ((s_imu[i].lastUpdate_ms != 0U) &&
                           ((now_ms - s_imu[i].lastUpdate_ms) <= IMU_TIMEOUT_MS)) ? 1U : 0U;
        s_imu[i].healthy = ImuManager_DataHealthCheck(&s_imu[i], now_ms);
    }
    ImuManager_IrqUnlock(primask);
}

static void ImuManager_UpdatePrimary(void)
{
    uint32_t primask;

    primask = ImuManager_IrqLock();
    if ((s_primary_id >= IMU_COUNT) || (s_imu[s_primary_id].healthy == 0U))
    {
        if (s_imu[IMU_ID_BMI088].healthy != 0U)
        {
            s_primary_id = IMU_ID_BMI088;
        }
        else if (s_imu[IMU_ID_ICM42688P].healthy != 0U)
        {
            s_primary_id = IMU_ID_ICM42688P;
        }
        else
        {
            s_primary_id = IMU_PRIMARY_DEFAULT_ID;
        }
    }
    ImuManager_IrqUnlock(primask);
}

static void ImuManager_CheckConsistency(void)
{
    float dgx;
    float dgy;
    float dgz;
    float dax;
    float day;
    float daz;
    float gyro_diff;
    float acc_diff;
    uint32_t primask;

    primask = ImuManager_IrqLock();
    if ((s_imu[IMU_ID_BMI088].healthy != 0U) && (s_imu[IMU_ID_ICM42688P].healthy != 0U))
    {
        dgx = s_imu[IMU_ID_BMI088].gx_cal_radps - s_imu[IMU_ID_ICM42688P].gx_cal_radps;
        dgy = s_imu[IMU_ID_BMI088].gy_cal_radps - s_imu[IMU_ID_ICM42688P].gy_cal_radps;
        dgz = s_imu[IMU_ID_BMI088].gz_cal_radps - s_imu[IMU_ID_ICM42688P].gz_cal_radps;
        dax = s_imu[IMU_ID_BMI088].ax_cal_mps2 - s_imu[IMU_ID_ICM42688P].ax_cal_mps2;
        day = s_imu[IMU_ID_BMI088].ay_cal_mps2 - s_imu[IMU_ID_ICM42688P].ay_cal_mps2;
        daz = s_imu[IMU_ID_BMI088].az_cal_mps2 - s_imu[IMU_ID_ICM42688P].az_cal_mps2;
        gyro_diff = sqrtf((dgx * dgx) + (dgy * dgy) + (dgz * dgz));
        acc_diff = sqrtf((dax * dax) + (day * day) + (daz * daz));

        if ((gyro_diff > IMU_GYRO_DIFF_LIMIT_RADPS) || (acc_diff > IMU_ACC_DIFF_LIMIT_MPS2))
        {
            s_consistency_error_count[IMU_ID_BMI088]++;
            s_consistency_error_count[IMU_ID_ICM42688P]++;
        }
    }
    ImuManager_IrqUnlock(primask);
}

static void ImuManager_ReinitOfflineImu(uint32_t now_ms)
{
    if ((s_imu[IMU_ID_BMI088].online == 0U) &&
        ((now_ms - s_last_reinit_try_ms[IMU_ID_BMI088]) >= IMU_REINIT_INTERVAL_MS))
    {
        s_last_reinit_try_ms[IMU_ID_BMI088] = now_ms;
        (void)ImuBmi088_Init();
    }

    if ((s_imu[IMU_ID_ICM42688P].online == 0U) &&
        ((now_ms - s_last_reinit_try_ms[IMU_ID_ICM42688P]) >= IMU_REINIT_INTERVAL_MS))
    {
        s_last_reinit_try_ms[IMU_ID_ICM42688P] = now_ms;
        (void)ImuIcm42688p_Init();
    }
}

static void ImuManager_CalResetAccum(void)
{
    memset(s_cal_accum, 0, sizeof(s_cal_accum));
}

static void ImuManager_CalCollect(void)
{
    uint8_t i;
    uint32_t primask;

    primask = ImuManager_IrqLock();
    for (i = 0U; i < IMU_COUNT; i++)
    {
        if ((s_imu[i].online != 0U) && (s_imu[i].data_valid != 0U) && (ImuManager_DataFinite(&s_imu[i]) != 0U))
        {
            s_cal_accum[i].acc_sum[0] += s_imu[i].ax_meas_mps2;
            s_cal_accum[i].acc_sum[1] += s_imu[i].ay_meas_mps2;
            s_cal_accum[i].acc_sum[2] += s_imu[i].az_meas_mps2;
            s_cal_accum[i].gyro_sum[0] += s_imu[i].gx_meas_radps;
            s_cal_accum[i].gyro_sum[1] += s_imu[i].gy_meas_radps;
            s_cal_accum[i].gyro_sum[2] += s_imu[i].gz_meas_radps;
            s_cal_accum[i].sample_count++;
        }
    }
    ImuManager_IrqUnlock(primask);
}

static void ImuManager_CalFinish(void)
{
    uint8_t i;
    uint8_t success = 0U;
    float mean_acc[3];
    float mean_gyro[3];
    uint32_t primask;

    primask = ImuManager_IrqLock();
    for (i = 0U; i < IMU_COUNT; i++)
    {
        if ((s_imu[i].online != 0U) && (s_cal_accum[i].sample_count >= IMU_STATIC_CAL_MIN_SAMPLES))
        {
            mean_acc[0] = (float)(s_cal_accum[i].acc_sum[0] / (double)s_cal_accum[i].sample_count);
            mean_acc[1] = (float)(s_cal_accum[i].acc_sum[1] / (double)s_cal_accum[i].sample_count);
            mean_acc[2] = (float)(s_cal_accum[i].acc_sum[2] / (double)s_cal_accum[i].sample_count);
            mean_gyro[0] = (float)(s_cal_accum[i].gyro_sum[0] / (double)s_cal_accum[i].sample_count);
            mean_gyro[1] = (float)(s_cal_accum[i].gyro_sum[1] / (double)s_cal_accum[i].sample_count);
            mean_gyro[2] = (float)(s_cal_accum[i].gyro_sum[2] / (double)s_cal_accum[i].sample_count);

            s_imu[i].acc_bias_mps2[0] = mean_acc[0] - IMU_STATIC_ACC_REF_X_MPS2;
            s_imu[i].acc_bias_mps2[1] = mean_acc[1] - IMU_STATIC_ACC_REF_Y_MPS2;
            s_imu[i].acc_bias_mps2[2] = mean_acc[2] - IMU_STATIC_ACC_REF_Z_MPS2;
            s_imu[i].gyro_bias_radps[0] = mean_gyro[0];
            s_imu[i].gyro_bias_radps[1] = mean_gyro[1];
            s_imu[i].gyro_bias_radps[2] = mean_gyro[2];
            s_imu[i].calibrated = 1U;
            ImuManager_DataApplyBias(&s_imu[i]);
            success = 1U;
        }
        else
        {
            memset(s_imu[i].acc_bias_mps2, 0, sizeof(s_imu[i].acc_bias_mps2));
            memset(s_imu[i].gyro_bias_radps, 0, sizeof(s_imu[i].gyro_bias_radps));
            s_imu[i].calibrated = 0U;
            ImuManager_DataApplyBias(&s_imu[i]);
        }
    }
    s_cal_state = (success != 0U) ? IMU_CAL_DONE : IMU_CAL_FAILED;
    ImuManager_IrqUnlock(primask);
}

static void ImuManager_CalUpdate(uint32_t now_ms)
{
    switch (s_cal_state)
    {
    case IMU_CAL_WAIT_STILL:
        if ((now_ms - s_cal_state_start_ms) >= IMU_STATIC_CAL_WAIT_MS)
        {
            ImuManager_CalResetAccum();
            s_cal_state_start_ms = now_ms;
            s_cal_state = IMU_CAL_COLLECTING;
        }
        break;

    case IMU_CAL_COLLECTING:
        ImuManager_CalCollect();
        if ((now_ms - s_cal_state_start_ms) >= IMU_STATIC_CAL_DURATION_MS)
        {
            ImuManager_CalFinish();
        }
        break;

    default:
        break;
    }
}

void ImuManager_Init(void)
{
    uint8_t i;
    uint32_t primask;

    primask = ImuManager_IrqLock();
    memset(s_imu, 0, sizeof(s_imu));
    memset(s_consistency_error_count, 0, sizeof(s_consistency_error_count));
    memset(s_last_reinit_try_ms, 0, sizeof(s_last_reinit_try_ms));
    ImuManager_CalResetAccum();
    for (i = 0U; i < IMU_COUNT; i++)
    {
        s_imu[i].imu_id = i;
    }
    s_primary_id = IMU_PRIMARY_DEFAULT_ID;
    s_cal_state = IMU_CAL_IDLE;
    s_initialized = 1U;
    ImuManager_IrqUnlock(primask);

    (void)ImuBmi088_Init();
    (void)ImuIcm42688p_Init();
}

void ImuManager_Update(void)
{
    uint32_t now_ms;

    if (s_initialized == 0U)
    {
        return;
    }

    ImuManager_UpdateOne(IMU_ID_BMI088, ImuBmi088_Read);
    ImuManager_UpdateOne(IMU_ID_ICM42688P, ImuIcm42688p_Read);
    now_ms = HAL_GetTick();
    ImuManager_UpdateHealth(now_ms);
    ImuManager_ReinitOfflineImu(now_ms);
    ImuManager_CheckConsistency();
    ImuManager_UpdatePrimary();
    ImuManager_CalUpdate(now_ms);
}

uint8_t ImuManager_GetImu(uint8_t imu_id, SensorsImuData_t *out)
{
    uint32_t primask;

    if ((imu_id >= IMU_COUNT) || (out == NULL))
    {
        return 0U;
    }

    primask = ImuManager_IrqLock();
    *out = s_imu[imu_id];
    ImuManager_IrqUnlock(primask);

    return out->online;
}

uint8_t ImuManager_GetPrimaryImu(SensorsImuData_t *out)
{
    return ImuManager_GetImu(ImuManager_GetPrimaryId(), out);
}

uint8_t ImuManager_IsOnline(uint8_t imu_id)
{
    SensorsImuData_t data;

    return ImuManager_GetImu(imu_id, &data);
}

uint8_t ImuManager_IsPrimaryOnline(void)
{
    return ImuManager_IsOnline(ImuManager_GetPrimaryId());
}

uint8_t ImuManager_GetPrimaryId(void)
{
    uint8_t id;
    uint32_t primask;

    primask = ImuManager_IrqLock();
    id = s_primary_id;
    ImuManager_IrqUnlock(primask);

    return id;
}

void ImuManager_SetPrimaryId(uint8_t imu_id)
{
    uint32_t primask;

    if (imu_id >= IMU_COUNT)
    {
        return;
    }

    primask = ImuManager_IrqLock();
    s_primary_id = imu_id;
    ImuManager_IrqUnlock(primask);
}

void ImuManager_StartStaticCalibration(void)
{
    uint32_t primask;

    primask = ImuManager_IrqLock();
    ImuManager_CalResetAccum();
    s_cal_state = IMU_CAL_WAIT_STILL;
    s_cal_state_start_ms = HAL_GetTick();
    ImuManager_IrqUnlock(primask);
}

void ImuManager_CancelStaticCalibration(void)
{
    uint32_t primask;

    primask = ImuManager_IrqLock();
    s_cal_state = IMU_CAL_IDLE;
    ImuManager_CalResetAccum();
    ImuManager_IrqUnlock(primask);
}

uint8_t ImuManager_IsCalibrating(void)
{
    ImuCalState_t state = ImuManager_GetCalibrationState();

    return ((state == IMU_CAL_WAIT_STILL) || (state == IMU_CAL_COLLECTING)) ? 1U : 0U;
}

uint8_t ImuManager_IsCalibrated(uint8_t imu_id)
{
    SensorsImuData_t data;

    if (ImuManager_GetImu(imu_id, &data) == 0U)
    {
        return 0U;
    }

    return data.calibrated;
}

ImuCalState_t ImuManager_GetCalibrationState(void)
{
    ImuCalState_t state;
    uint32_t primask;

    primask = ImuManager_IrqLock();
    state = s_cal_state;
    ImuManager_IrqUnlock(primask);

    return state;
}
