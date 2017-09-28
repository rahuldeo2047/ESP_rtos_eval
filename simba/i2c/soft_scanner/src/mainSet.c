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

#define SCL_PIN &pin_d4_dev
#define SDA_PIN &pin_d5_dev

struct i2c_soft_driver_t i2c;

struct mpu6050_basic_setting
{
    uint8_t address;
    uint8_t data;
    int8_t errmpu;
};

struct mpu6050_basic_setting settings_1[12] =
{
  { 25,  1 , -1}, 	//  sample rate divider: sample rate = mstrClock / (1 +  divider)
  { 26,  6			, -2},  //  DLPF set.  (0 = 8kHz master clock else 1kHz master clock)
  { 27,  24					, -3},  //  gyro full scale range
  { 28,  0					, -4},  //  accel full scale range
  { 31,  0b00000000			, -5},	//  no motion detect
  { 35,  0b00000000			, -6},	//  no FIFO
  { 36,  0b00000000			, -7},	//  no mstr I2C
  { 55,  0b01110000			, -8},	//	configure interrupt  -- on when data ready, off on data read
  { 56,  0b00000001			, -9},	//	interrupt on
  { 106, 0b00000000		  , -10},	//  no silly stuff
  { 107, 0b00000001			, -11},	//  no sleep and clock off gyro_X
  { 108, 0b00000000			, -12}   //  no goofball sleep mode
};

ssize_t res = 0 ;
uint8_t buf[14];

int main()
{
  int address;
  int number_of_slaves;

  res = res;

  sys_start();
  i2c_soft_module_init();

  i2c_soft_init(&i2c, SCL_PIN, SDA_PIN, 50000, 1000000, 1000);
  i2c_soft_start(&i2c);

  std_printf(FSTR("Scanning the i2c bus for slaves...\r\n"
  "\r\n"));

  number_of_slaves = 0;

  for (address = 0; address < 127; address++) {
    if (i2c_soft_scan(&i2c, address) == 1) {
      std_printf(FSTR("Found slave with address 0x%x.\r\n"), address);
      number_of_slaves++;
    }
  }

  std_printf(FSTR("\r\n"
  "Scan complete. Found %d slaves.\r\n"), number_of_slaves);

  buf[0] = 0x6B;
  res = i2c_soft_write(
    &i2c,
    0x68,
    &buf[0],
    1
  );

  std_printf(FSTR("\r\n"
  "status  %d.\r\n"),  res);


  for (int i = 0 ; i < 12 ; i++)
	{

		res = i2c_soft_write(
			&i2c,
      0x68,
			&settings_1[i].address,
			settings_1[i].data
		);

		if (res != 0)
		{
			//DLOG(WARNING, "Failed: %d with %d.\r\n", settings[i].errmpu, res);
			//return (res);
      std_printf(FSTR("\r\n"
      "Failed: %d with %d.\r\n"), settings_1[i].errmpu, res);

		}
	}

 // Assuming mpu is configured already
  while(1)
  {

    buf[0] = 0x3B;
    res = i2c_soft_write(
      &i2c,
      0x68,
      &buf[0],
      1
    );

    std_printf(FSTR("\r\n"
    "res %d Data %d.\r\n"), res, buf[0] << 8 | buf[1]);

    buf[0] = 0;

    res = i2c_soft_read(
      &i2c,
      0x68,
      buf,
      14
    );

    std_printf(FSTR("\r\n"
    "res %d Data %d.\r\n"), res, buf[0] << 8 | buf[1]);

    thrd_sleep(1);
  }
  return (0);
}
