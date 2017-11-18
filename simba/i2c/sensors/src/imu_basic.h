
#ifndef __IMU_BASIC_H__
#define __IMU_BASIC_H__


#include "simba.h"
#include "mpu6050_basic.h"

struct imu_thrd_data_t
{
  uint32_t seq;
  struct time_t ts;
  struct sMPUDATA_t mpudata;
  struct Vec3 YPR;
};

#define IMU_THRD_NAME ("imu_thrd")
inline char * imu_thrd_get_thrd_name()
{
  return (IMU_THRD_NAME);
}

inline int imu_thrd_get_channel_id(int *chnid_p)
{
   *chnid_p = 0x01;
   return (0);
}

void *imu_thrd(void *arg_p);



#endif // __IMU_BASIC_H__
