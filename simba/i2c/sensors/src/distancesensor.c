
#include "simba.h"

#include "distancesensor.h"

#include "common.h"

#include <time.h>

#define DS_DEEP_DEBUG 1
#define ds_deep_debug_log if(DS_DEEP_DEBUG)std_printf
#define DS_DEBUG ds_deep_debug_log(OSTR("DEBUG %s(): %d\r\n"),__func__, __LINE__);
#define DS_DEBUG_VAL(x) ds_deep_debug_log(OSTR("DEBUG %s(): %d %f\r\n"),__func__, __LINE__, x);

static struct pin_driver_t pinecho;
static struct pin_driver_t pintrig;

static int intialize_distancesensor(void)
{

  int pin_module_init();

  BTASSERT(pin_init(&pinecho, &DISTANCESENSOR_ULTRASONIC_ECHO_PIN,    PIN_INPUT ) == 0);
  BTASSERT(pin_init(&pintrig, &DISTANCESENSOR_ULTRASONIC_TRIGGER_PIN, PIN_OUTPUT) == 0);

  BTASSERT(pin_write(&pinecho, 1) == 0);
  BTASSERT(pin_write(&pintrig, 0) == 0);


  return (0);
}

static int distancesensor_set_trigger(int state)
{
  BTASSERT(pin_write(&pintrig, state) == 0);
  return (0);
}

static int distancesensor_get_echo_status()
{
  return (pin_read(&pinecho));
}

static int distancesensor_get_echo_time(int32_t *ts)
{

  #define DISTANCESENSOR_PING_MAX_TIME (100000) // uS  50uS == 1 cm

  static bool distancesensor_trigger_status, distancesensor_echo_status;

  struct time_t starttime, endtime, deltatime;
  int32_t ret;

  ret = 0;

  time_get(&starttime);

  //ds_deep_debug_log(OSTR("DEBUG %s: %d\r\n"),__func__, __LINE__);
  DS_DEBUG

  if(1 == distancesensor_trigger_status) // earlier ping in progress
  {
    *ts = 0;
    return -1;
  }

  DS_DEBUG

  distancesensor_trigger_status = 1;

  distancesensor_set_trigger(0);
  thrd_sleep_us(1);
  //DS_DEBUG_VAL((float)distancesensor_get_echo_status());

  distancesensor_set_trigger(1); // Is it a voltage issue 3.3v
  thrd_sleep_us(10);
  //DS_DEBUG_VAL((float)distancesensor_get_echo_status());

  distancesensor_set_trigger(0);
  //DS_DEBUG_VAL((float)distancesensor_get_echo_status());

  distancesensor_trigger_status = 0;

  if(1 == distancesensor_echo_status) // earlier ping in progress
  {
    *ts = 0;
    return -2;
  }

  distancesensor_echo_status = 1;
  //starttime = time_micros();
  time_get(&starttime);

  ret = distancesensor_get_echo_status();
  while(ret == 0)
  {
    ret = distancesensor_get_echo_status();

    time_get(&endtime);
    //deltatime = time_micros_elapsed(starttime, endtime);
    time_subtract(&deltatime, &endtime, &starttime); // do not use remote_addr

    if((deltatime.nanoseconds/1000)>=DISTANCESENSOR_PING_MAX_TIME) // out of time and beyond supported range
    {
      DS_DEBUG_VAL((float)ret);
      DS_DEBUG_VAL((float)deltatime.seconds);
      DS_DEBUG_VAL((float)deltatime.nanoseconds);

      distancesensor_echo_status = 0;
      *ts = (deltatime.nanoseconds/1000);
      return (-3);
      break;
    }
  }

  time_get(&starttime);

  // wait till the echo is low
  //do {
    ret = distancesensor_get_echo_status();
    while(ret==1)
    {

    //endtime = time_micros();
    time_get(&endtime);

    //deltatime = time_micros_elapsed(starttime, endtime);
    time_subtract(&deltatime, &endtime, &starttime); // do not use ret

    //DS_DEBUG_VAL((float)deltatime.nanoseconds);

    ret = distancesensor_get_echo_status(); // the return ret must not be used in other status

    if((deltatime.nanoseconds/1000)>=DISTANCESENSOR_PING_MAX_TIME) // out of time and beyond supported range
    {
      DS_DEBUG_VAL((float)ret);
      DS_DEBUG_VAL((float)deltatime.seconds);
      DS_DEBUG_VAL((float)deltatime.nanoseconds);

      distancesensor_echo_status = 0;
      *ts = (deltatime.nanoseconds/1000);
      return (-4);
      break;
    }

  } //while(ret==0);

  distancesensor_echo_status = 0;

  //endtime = time_micros();
  //time_get(&endtime);

  DS_DEBUG_VAL((float)ret);

  //deltatime = time_micros_elapsed(starttime, endtime);
  //ret = time_subtract(&deltatime, &endtime, &starttime);


  //deltatime = time_micros_elapsed(starttime, endtime);
  *ts = (deltatime.nanoseconds/1000);

  DS_DEBUG_VAL((float)deltatime.seconds);
  DS_DEBUG_VAL((float)deltatime.nanoseconds);

  return (0);
}

static int distancesensor_get_echo_distance_in_cm(float *d)
{

  #define DISTANCESENSOR_PING_CONVERTION_TO_CM (50.0f) // uS US_ROUNDTRIP_CM 50

  int res;
  int32_t ts;

  res = distancesensor_get_echo_time(&ts);

  DS_DEBUG_VAL((float)res);
  DS_DEBUG_VAL((float)ts);

  *d = 0;

  if(0==res)
    *d = (float)ts / DISTANCESENSOR_PING_CONVERTION_TO_CM;


  return (0);

}

void *distancesensor_thrd(void *arg_p)
{

  thrd_set_name(distancesensor_thrd_get_thrd_name());

  int res;
  struct bus_info_t * bus_info;
  float disctance_in_cm;

  bus_info = (struct bus_info_t *)arg_p;
  bus_info = bus_info;

  res = intialize_distancesensor();
  res = res;

  thrd_sleep_ms(3000);

  while(true)
  {

    distancesensor_get_echo_distance_in_cm(&disctance_in_cm);

    thrd_sleep_ms(1000);

    std_printf(OSTR("Read distance data %f \r\n"), disctance_in_cm);

    //if(queue_write((bus_info->queue), &imudata, sizeof(imudata)) > 0)// sizeof(imudata))
    {

    }
  }

}
