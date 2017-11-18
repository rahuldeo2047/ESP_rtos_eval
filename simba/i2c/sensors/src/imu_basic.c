
#include "simba.h"
//#include "config.h"

#include "mpu6050_basic.h"

#include "imu_basic.h"

#include "common.h"


void *imu_thrd(void *arg_p)
{

  thrd_set_name(imu_thrd_get_thrd_name());

  struct mpu6050_basic_driver_t mpu6050basic_dev;
  struct mpu6050_basic_transport_i2c_t mpu6050basic_transport;
  struct mpu6050_basic_config_initial mpu6050basic_config;

  // Profliling
  struct time_t uptime, uptimeRes;
  struct time_t luptime;
  struct time_t time1, timeRes;
  struct time_t time2;


  struct time_t timestamp, timestamplast, timestampdelta;
  uint32_t dt_us, tdt_us;

  struct bus_info_t *bus_info_p;
  bus_info_p = (struct bus_info_t *)arg_p;

  int my_id = -1;
  imu_thrd_get_channel_id(&my_id);


  #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
  struct i2c_driver_t i2c;

  #else

  #define SCL_PIN &pin_d4_dev
  #define SDA_PIN &pin_d5_dev

  struct i2c_soft_driver_t i2c;

  #endif

  struct imu_thrd_data_t imudata;

  int address;
  int number_of_slaves;
  int res;

  mpu6050basic_config.accelRange    = 0;
  mpu6050basic_config.filterLevel   = 6;
  mpu6050basic_config.gyroRange     = 3;
  mpu6050basic_config.sampleRate    = 400;

  mpu6050basic_dev.config.config = mpu6050basic_config;

  imudata.seq = 0;

  //timer_module_init();
  //timer_init(struct timer_t *self_p, const struct time_t *timeout_p, timer_callback_t callback, void *arg_p, int flags)
 
  #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
  std_printf(FSTR("Hardware I2C is being used.\r\n"
  "\r\n"));
  i2c_module_init();

  #else
  std_printf(FSTR("Software I2C is being used.\r\n"
  "\r\n"));
  i2c_soft_module_init();

  #endif

  mpu6050_basic_module_init();

  //i2c_init(&i2c, &i2c_0_dev, I2C_BAUDRATE_100KBPS, -1);
  #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)

  i2c_init(&i2c, &i2c_2_dev, I2C_BAUDRATE_400KBPS, -1);
  i2c_start(&i2c);

  #else

  i2c_soft_init(&i2c, SCL_PIN, SDA_PIN, 50000, 1000000, 1000);
  i2c_soft_start(&i2c);

  #endif

  std_printf(FSTR("Scanning the i2c bus for slaves...\r\n"
  "\r\n"));

  number_of_slaves = 0;

  for (address = 0; address < 128; address++)
  {
    #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
    if (i2c_scan(&i2c, address) == 1)
    #else
    if (i2c_soft_scan(&i2c, address) == 1)
    #endif
    {
      std_printf(FSTR("Found slave with address 0x%x.\r\n"), address);
      number_of_slaves++;
    }
  }

  std_printf(FSTR("\r\n"
  "Scan complete. Found %d slaves.\r\n"), number_of_slaves);

  #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)

  if (i2c_scan(&i2c, MPU6050_BASIC_I2C_ADDRESS_0) == 1)

  #else

  if (i2c_soft_scan(&i2c, MPU6050_BASIC_I2C_ADDRESS_0) == 1)

  #endif
  {
    std_printf(FSTR("recheck Found slave with address 0x%x.\r\n"), MPU6050_BASIC_I2C_ADDRESS_0);
  }

  res = mpu6050_basic_transport_i2c_init(&mpu6050basic_transport,
    &i2c,
    MPU6050_BASIC_I2C_ADDRESS_0);

    std_printf(FSTR("\r\n"
    "transport initialization status: %d.\r\n"), res);


    res = mpu6050_basic_init(&mpu6050basic_dev, &mpu6050basic_transport.base);

    std_printf(FSTR("\r\n"
    "mpu initialization status: %d.\r\n"), res);


    res = mpu6050_basic_start(&mpu6050basic_dev, &imudata.mpudata);

    if (res != 0) {
      std_printf(OSTR("Failed to start the device.\r\n"));
      while(1); // trap or ddestroy the thread

    }


    int seq = 0;
    while (1)
    {

      //time_get(&time1);
      //sys_uptime(&uptime);

      // thread sleep is not accurate
      //thrd_sleep_us(1);//mpu6050basic_dev.config._internal._samplePeriod);
      thrd_sleep_ms(5);
      //time_busy_wait_us(mpu6050basic_dev.config._internal._samplePeriod);

      /* Read accelerometer, temparature and gyro data from mpu6050 . */
      res = mpu6050_basic_read(&mpu6050basic_dev, &imudata.mpudata);


      if (res != 0)
      {
        std_printf(OSTR("Read failed with %d.\r\n"),
        res
      );
      continue;
    }
    else
    {
      time_get(&timestamp);
      time_subtract(&timestampdelta, &timestamp,  &timestamplast);
      // //dt_us = (timestamp.nanoseconds/1000UL) - (timestamplast.nanoseconds/1000UL); // uS
      dt_us = timestampdelta.nanoseconds/1000UL; // corrections
      //
      dt_us = dt_us;

      seq++;

      imudata.seq = seq;
      imudata.ts = timestamp;
      //imudata.

      // TODO: Time it for the sample time
      //if(dt_us >= mpu6050basic_dev.config._internal._samplePeriod/2)
      {

        timestamplast = timestamp;
        //
        // if(timestampdelta.seconds > 0)
        // {
        //   std_printf(OSTR("warn: very late calculation %lu.%lu.\r\n")
        //   , timestampdelta.seconds, timestampdelta.nanoseconds);
        // }

        res = mpu6050_motion_calc(&mpu6050basic_dev, &imudata.mpudata, &imudata.YPR
          , dt_us);// mpu6050basic_dev.config._internal._samplePeriod);

        // sys_uptime(&uptime);
        //
        // time_subtract(&timeRes,   &time2,  &time1);
        // time_subtract(&uptimeRes, &uptime, &luptime);

        imudata.ts = timestamp;//delta;

        if (res != 0)
        {
          std_printf(
            OSTR("Read failed with %d.\r\n"),
            res
          );
          continue;
        }
        else
        {

          //if(0==(cnt%10))
          {
            imudata.seq++;
            bus_write(bus_info_p->bus, my_id, &imudata, sizeof(imudata));
          }


          //  if(0==(cnt%10))
          //   std_printf(OSTR("Read data %lu.%lu A[%d %d %d], Tmp:%f, G[%d %d %d], YPR[%f %f %f] \r\n"),
          //   timeRes.seconds,
          //   timeRes.nanoseconds,
          //   mpudata.AcX,
          //   mpudata.AcY,
          //   mpudata.AcZ,
          //   /*mpudata.Tmp, */(float)(mpudata.Tmp) * (1.0f / 340.0f) + 36.53f,
          //   mpudata.GyX,
          //   mpudata.GyY,
          //   mpudata.GyZ,
          //   YPR.x,
          //   YPR.y,
          //   YPR.z
          // );
        }
      }

    }

  }


}
