/**
* @section License
*
* The MIT License (MIT)
*
* Copyright (c) 2014-2017, Erik Moqvist
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* This file is part of the Simba project.
*/

#include "simba.h"
//#include "config.h"


#include "imu_basic.h"
#include "communication.h"
//#include "mpu6050_basic.h"
//#
#include "common.h"

static THRD_STACK(imu_basic_stack, 1024);
static THRD_STACK(communication_stack, 10*1024);


// Wifi test
//

int main()
{

  sys_start();


  log_object_print(NULL,
    LOG_INFO,
    OSTR("IMU basic attempting to start...\r\n"));


    BTASSERT(bus_module_init() == 0);

    //
    struct bus_info_t bus_info;

    struct bus_t bus;
    struct bus_listener_t bus_listener[2]; // number of listeners wifi and print
    struct queue_t queue;

    bus_info.bus = &bus;
    //bus_info.bus_listener = &bus_listener;
    bus_info.queue = &queue;

    struct imu_thrd_data_t imudata;
    struct imu_thrd_data_t imudataBUF[10];

    int chanid_imu = -1;
    imu_thrd_get_channel_id( &chanid_imu );

    BTASSERT(queue_init(bus_info.queue, &imudataBUF[0], sizeof(imudataBUF)) == 0);
    BTASSERT(bus_init(bus_info.bus) == 0);
    BTASSERT(bus_listener_init(&bus_listener[0], chanid_imu, bus_info.queue) == 0);

    /* Attach-detach a channel. */
    BTASSERT(bus_attach(bus_info.bus, &bus_listener[0]) == 0);

    int chanid_wifi= -1;
    comm_thrd_get_channel_id( &chanid_wifi );

    //BTASSERT(queue_init(bus_info.queue, &imudataBUF[0], sizeof(imudataBUF)) == 0);
    //BTASSERT(bus_init(bus_info.bus) == 0);
    BTASSERT(bus_listener_init(&bus_listener[1], chanid_wifi, bus_info.queue) == 0);

    /* Attach-detach a channel. */
    BTASSERT(bus_attach(bus_info.bus, &bus_listener[1]) == 0);

    // thrd_spawn .... mpu task
    BTASSERT(thrd_spawn(imu_thrd,
      (void*)&bus_info,
      20,
      imu_basic_stack,
      sizeof(imu_basic_stack)) != NULL);

    // wifi
    //test_station();
    BTASSERT(thrd_spawn(comm_thrd,
      (void*)&bus_info,
      21,
      communication_stack,
      sizeof(communication_stack)) != NULL);


      while(1)
      {
        // read bus

        if(queue_read(&queue, &imudata, sizeof(imudata)) > 0)// sizeof(imudata))
        {
          //imudata = imudataBUF[0]; // COULD BE THE LAST ONE
          if (0 == imudata.seq%100) // ignore few data and print slow
          {
            std_printf(OSTR("Read data %lu: %lu.%lu A[%d %d %d], Tmp:%f, G[%d %d %d], YPR[%f %f %f] \r\n"),
            imudata.seq,
            imudata.ts.seconds,
            imudata.ts.nanoseconds,
            imudata.mpudata.AcX,
            imudata.mpudata.AcY,
            imudata.mpudata.AcZ,
            /*mpudata.Tmp, */(float)(imudata.mpudata.Tmp) * (1.0f / 340.0f) + 36.53f,
            imudata.mpudata.GyX,
            imudata.mpudata.GyY,
            imudata.mpudata.GyZ,
            imudata.YPR.x,
            imudata.YPR.y,
            imudata.YPR.z
          );
        }
      }

    }



    return (0);
  }
