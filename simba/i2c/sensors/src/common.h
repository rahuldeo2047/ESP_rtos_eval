
#ifndef __COMMON_H__
#define __COMMON_H__



#include "imu_basic.h"
#include "mpu6050_basic.h"

struct bus_info_t
{
   struct bus_t *bus;
   //struct bus_listener_t *bus_listener;
   struct queue_t *queue;
};



#endif
