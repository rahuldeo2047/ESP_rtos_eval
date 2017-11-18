#ifndef __DISTANCESENSOR_H__
#define __DISTANCESENSOR2_H__

#include "simba.h"

#define DISTANCESENSOR_ULTRASONIC_TRIGGER_PIN (pin_D6_dev) //(pin_d4_dev)
#define DISTANCESENSOR_ULTRASONIC_ECHO_PIN    (pin_D5_dev) //(pin_d5_dev)

#define DISTANCESENSOR_THRD_NAME ("disctancesensor_thrd")
inline char * distancesensor_thrd_get_thrd_name()
{
  return (DISTANCESENSOR_THRD_NAME);
}

inline int distancesensor_thrd_get_channel_id(int *chnid_p)
{
   *chnid_p = 0x03;
   return (0);
}

void *distancesensor_thrd(void *arg_p);



#endif // __DISTANCESENSOR_H__
