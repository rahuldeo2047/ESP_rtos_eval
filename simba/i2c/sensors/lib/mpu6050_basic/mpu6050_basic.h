

#ifndef __DRIVERS_SENSORS_MPU6050_H__
#define __DRIVERS_SENSORS_MPU6050_H__


#include "simba.h"
#include "Math3D.h" // Any 3D vector lib
//#include "MadgwickAHRS.h"

#define MPU6050_BASIC_I2C_ADDRESS_0                             (0x68)
#define MPU6050_BASIC_I2C_ADDRESS_1                             (0x69)
#define MPU6050_BASIC_I2C_ADDRESS_AUTOMATIC                     (0xff)

#define MPU6050_BASIC_REG_WHO_AM_I                              (0x75)

#define MPU6050_BASIC_REG_PWR_MGMT_1                            (0x6B)

#define MPU6050_BASIC_REG_DATA_RDY_INT_STATUS                   (0x3A)

#define MPU6050_BASIC_REG_ACCEL_XOUT_H                          (0x3B)

// #define MPU6050_BASIC_REG_ACCEL_XOUT_H                          (0x3B)
// #define MPU6050_BASIC_REG_ACCEL_XOUT_L                          (0x3C)
// #define MPU6050_BASIC_REG_ACCEL_YOUT_H                          (0x3D)
// #define MPU6050_BASIC_REG_ACCEL_YOUT_L                          (0x3E)
// #define MPU6050_BASIC_REG_ACCEL_ZOUT_H                          (0x3F)
// #define MPU6050_BASIC_REG_ACCEL_ZOUT_L                          (0x40)
// #define MPU6050_BASIC_REG_TEMP_OUT_H                            (0x41)
// #define MPU6050_BASIC_REG_TEMP_OUT_L                            (0x42)
// #define MPU6050_BASIC_REG_GYRO_XOUT_H                           (0x43)
// #define MPU6050_BASIC_REG_GYRO_XOUT_L                           (0x44)
// #define MPU6050_BASIC_REG_GYRO_YOUT_H                           (0x45)
// #define MPU6050_BASIC_REG_GYRO_YOUT_L                           (0x46)
// #define MPU6050_BASIC_REG_GYRO_ZOUT_H                           (0x47)
// #define MPU6050_BASIC_REG_GYRO_ZOUT_L                           (0x48)

struct mpu6050_basic_setting
{
    uint8_t address;
    uint8_t data;
    int8_t errmpu;
};

struct sMPUDATA_t
{

  uint32_t timestamp;
  /*int16_t*/
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

};

struct mpu6050_basic_config_internal
{
  uint16_t _GyroClk;
  uint8_t  _sampleRateDiv;
  uint8_t  _gFSR;
  uint8_t  _aFSR;
  uint32_t _samplePeriod; // uS
  float accelToG, gyroToRad;
};

struct mpu6050_basic_config_initial
{
  uint16_t sampleRate;
  uint8_t filterLevel;
  uint8_t gyroRange;
  uint8_t accelRange;
};

struct mpu6050_basic_motion
{
  struct Quat AttitudeEstimateQuat;
};

struct mpu6050_basic_transport_protocol_t;

struct mpu6050_basic_transport_t
{
    struct mpu6050_basic_transport_protocol_t *protocol_p;
};

struct mpu6050_basic_transport_i2c_t
{
    struct mpu6050_basic_transport_t base;
    #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
    struct i2c_driver_t *i2c_p;
    #else
    struct i2c_soft_driver_t *i2c_p;
    #endif
    int i2c_address;
};

struct mpu6050_basic_config
{
	struct mpu6050_basic_config_initial config;
  struct sMPUDATA_t bias;
  int bias_precalculated;
  struct mpu6050_basic_config_internal _internal;
};

struct mpu6050_basic_driver_t
{
    struct mpu6050_basic_transport_t *transport_p;
    struct mpu6050_basic_config config;
    struct mpu6050_basic_motion motion;

#if CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK > -1
    struct log_object_t log;
#endif
};

int mpu6050_basic_transport_i2c_init(
	struct mpu6050_basic_transport_i2c_t *self_p,
  #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
  struct i2c_driver_t *i2c_p,
  #else
  struct i2c_soft_driver_t *i2c_p,
  #endif
	int i2c_address
);

int mpu6050_basic_module_init();

int mpu6050_basic_init(struct mpu6050_basic_driver_t *self_p,
                struct mpu6050_basic_transport_t *transport_p);

int mpu6050_basic_start(struct mpu6050_basic_driver_t *self_p, struct sMPUDATA_t *data_p);

int mpu6050_basic_read(
	struct mpu6050_basic_driver_t *self_p,
	struct sMPUDATA_t *data_p
);

int mpu6050_basic_motion_agzero(struct mpu6050_basic_driver_t *self_p, struct sMPUDATA_t *data_p);
int mpu6050_motion_calc(struct mpu6050_basic_driver_t *self_p, struct sMPUDATA_t *data_p, struct Vec3 *YPR, const uint32_t dt_us); // must be called every config.sampleRate duration uS

#endif // __DRIVERS_SENSORS_MPU6050_H__
