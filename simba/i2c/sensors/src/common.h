
#ifndef __COMMON_H__
#define __COMMON_H__



#include "imu_basic.h"
#include "mpu6050_basic.h"
#include "/home/rahuldeo/ATOM/ATOM_DRONE/ESP8266_D1/src/data2.h"

struct bus_info_t
{
   struct bus_t *bus;
   //struct bus_listener_t *bus_listener;
   struct queue_t *queue;
};



#endif
