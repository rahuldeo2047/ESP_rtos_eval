

#ifndef __DRIVERS_SENSORS_MPU6050_H__
#define __DRIVERS_SENSORS_MPU6050_H__


#include "simba.h"

#define MPU6050_BASIC_I2C_ADDRESS_0                             (0x68)
#define MPU6050_BASIC_I2C_ADDRESS_1                             (0x69)
#define MPU6050_BASIC_I2C_ADDRESS_AUTOMATIC                     (0xff)

#define MPU6050_BASIC_REG_WHO_AM_I                              (0x75)

#define MPU6050_BASIC_REG_PWR_MGMT_1                            (0x6B)
#define MPU6050_BASIC_REG_ACCEL_XOUT_H                          (0x3B)

struct mpu6050_basic_setting
{
    uint8_t address;
    uint8_t data;
    int8_t errmpu;
};

struct mpu6050_basic_transport_protocol_t;

struct mpu6050_basic_transport_t
{
    struct mpu6050_basic_transport_protocol_t *protocol_p;
};

struct mpu6050_basic_transport_i2c_t
{
    struct mpu6050_basic_transport_t base;
    struct i2c_driver_t *i2c_p;
    int i2c_address;
};


struct sMPUDATA_t
{

  uint32_t timestamp;
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

};

struct mpu6050_basic_config_internal
{
  uint16_t _GyroClk;
  uint8_t  _sampleRateDiv;
  uint8_t  _gFSR;
  uint8_t  _aFSR;
  uint32_t _samplePeriod;
  float accelToG, gyroToRad;
};

struct mpu6050_basic_config_initial
{
  uint16_t sampleRate;
  uint8_t filterLevel;
  uint8_t gyroRange;
  uint8_t accelRange;
};

struct mpu6050_basic_config
{
	struct mpu6050_basic_config_initial config;
  struct sMPUDATA_t bias;
  struct mpu6050_basic_config_internal _internal;
};

struct mpu6050_basic_driver_t
{
    struct mpu6050_basic_transport_t *transport_p;
    struct mpu6050_basic_config config;

#if CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK > -1
    struct log_object_t log;
#endif
};

int mpu6050_basic_transport_i2c_init(
	struct mpu6050_basic_transport_i2c_t *self_p,
	struct i2c_driver_t *i2c_p,
	int i2c_address
);

int mpu6050_basic_module_init();

int mpu6050_basic_init(struct mpu6050_basic_driver_t *self_p,
                struct mpu6050_basic_transport_t *transport_p);

int mpu6050_basic_start(struct mpu6050_basic_driver_t *self_p);

int mpu6050_basic_read(
	struct mpu6050_basic_driver_t *self_p,
	struct sMPUDATA_t *data_p
);

#endif // __DRIVERS_SENSORS_MPU6050_H__
