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

#include "mpu6050_basic.h"

int main()
{

  struct mpu6050_basic_driver_t mpu6050basic_dev;
  struct mpu6050_basic_transport_i2c_t mpu6050basic_transport;
  struct mpu6050_basic_config_initial mpu6050basic_config;

  #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
  struct i2c_driver_t i2c;

  #else

  #define SCL_PIN &pin_d4_dev
  #define SDA_PIN &pin_d5_dev

  struct i2c_soft_driver_t i2c;

  #endif

  struct sMPUDATA_t mpudata;

  int address;
  int number_of_slaves;
  int res;

  mpu6050basic_config.accelRange    = 0;
  mpu6050basic_config.filterLevel   = 6;
  mpu6050basic_config.gyroRange     = 3;
  mpu6050basic_config.sampleRate    = 400;

  mpu6050basic_dev.config.config = mpu6050basic_config;

  sys_start();

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

  i2c_init(&i2c, &i2c_0_dev, I2C_BAUDRATE_400KBPS, -1);
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


    res = mpu6050_basic_start(&mpu6050basic_dev);

    if (res != 0) {
      std_printf(OSTR("Failed to start the device.\r\n"));
      return (res);
    }

    while (1)
    {
      thrd_sleep(1);

      /* Read temperature and pressure from the BMP280. */
      res = mpu6050_basic_read(&mpu6050basic_dev, &mpudata);

      if (res != 0)
      {
        std_printf(OSTR("Read failed with %d.\r\n"),
        res
      );
      continue;
    }
    else
    {
      std_printf(OSTR("Read data Ax:%d, %d, %d, Tmp:%d, Gx%d, %d, %d.\r\n"),
      mpudata.AcX,
      mpudata.AcY,
      mpudata.AcZ,
      mpudata.Tmp,
      mpudata.GyX,
      mpudata.GyY,
      mpudata.GyZ
    );

  }

}

return (0);
}
