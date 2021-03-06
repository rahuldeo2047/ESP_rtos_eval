


#include "simba.h"

#include "kernel/time.h"
#include "mpu6050_basic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func_part.h"
//#include "Math3D.h"


#define TEMP_SCALE  (1.0f / 340.0f)
#define TEMP_OFFSET 36.53f

#define ACCEL_BASE	2048.0f		// LSB per g   @ +/- 16g
#define GYRO_BASE		16.375f		// LSB per dps @ +/- 2000 deg/s

#define DEG_TO_RAD ((3.141592654f / 180.0f))
#define RAD_TO_DEG ((180.0f / 3.141592654f))

#if (CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK > -1)
#define DLOG(level, msg, ...)                                       \
log_object_print(&(self_p)->log, LOG_ ## level, OSTR(msg), ##__VA_ARGS__)
#else
#define DLOG(level, msg, ...)
#endif //  #if defined(DEBUG_MPU6050_BASIC)

#define DEEP_DEBUG 0
#define deep_debug_log if(DEEP_DEBUG)std_printf

static struct mpu6050_basic_driver_t *g_self_p;

struct mpu6050_basic_transport_protocol_t
{
  int (*start)(struct mpu6050_basic_driver_t *self_p);

  int (*read)
  (
    struct mpu6050_basic_driver_t *self_p,
    uint8_t address,
    uint8_t *buf_p,
    size_t size
  );

  int (*write)
  (
    struct mpu6050_basic_driver_t *self_p,
    uint8_t address,
    //struct mpu6050_basic_transport_i2c_t *transport_p;
    uint8_t data
  );
};

int mpu6050_basic_read_mpu(
  struct mpu6050_basic_driver_t *self_p,
  struct sMPUDATA_t *data_p
);

float temp_C(uint16_t tempRaw);

int mpu6050_basic_init(
  struct mpu6050_basic_driver_t *self_p,
  struct mpu6050_basic_transport_t *transport_p
)//uint16_t sampleRate, uint8_t filterLevel, uint8_t gyroRange, uint8_t accelRange)
{
  int ret;
  ret = 0;
  #if (CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK > -1)
  log_object_init(&self_p->log, "mpu6050basic", CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK);
  #endif

  self_p->transport_p = transport_p;

  // g_self_p = self_p; // for external libs

  // self_p->config.config.accelRange	=	0;
  // self_p->config.config.filterLevel	= 6;
  // self_p->config.config.gyroRange		= 3;
  // self_p->config.config.sampleRate  = 400;

  if(self_p->config.config.filterLevel > 0)
  {
    self_p->config._internal._GyroClk = 1000;
  }
  else
  {
    self_p->config._internal._GyroClk = 8000;
  }

  self_p->config._internal._sampleRateDiv
  = (uint8_t)((((float)(self_p->config._internal._GyroClk) / (float)(self_p->config.config.sampleRate)) - 1));

  self_p->config._internal._samplePeriod
  = (uint32_t)((((float)(self_p->config._internal._sampleRateDiv) + 1) * 1000000UL) / (float)(self_p->config._internal._GyroClk)); // time between samples (us)

  //self_p->config._internal._sampleRateDiv = (uint8_t)(((float)self_p->config._internal._GyroClk / (float)self_p->config.config.sampleRate) - 1);
  //self_p->config._internal._samplePeriod = (uint8_t)((float)(self_p->config._internal._sampleRateDiv + 1) * 1000000UL) / (float)self_p->config._internal._GyroClk; // time between samples (us)

  self_p->config._internal._gFSR = self_p->config.config.gyroRange  << 3;	// bitshift to correct position for settings register
  self_p->config._internal._aFSR = self_p->config.config.accelRange << 3;

  self_p->config._internal.accelToG  = 1.0f / (ACCEL_BASE * (float)(1 << (3 - self_p->config.config.accelRange)));		// constant to convert from raw int to float G

  self_p->config._internal.gyroToRad = DEG_TO_RAD / (GYRO_BASE * (float)(1 << (3 - self_p->config.config.gyroRange)));	// constant to convert from raw int to float rad/sec

  self_p->config.bias.timestamp = 0;

# if (CONFIG_MPU6050_DMP_ENABLE > 0)

# else

  self_p->config.bias.AcX = -1384;
  self_p->config.bias.AcY = -620;
  self_p->config.bias.AcZ = 16448;
  self_p->config.bias.Tmp = 0;
  self_p->config.bias.GyX = -7;
  self_p->config.bias.GyY = 31;
  self_p->config.bias.GyZ = 9;

# endif

  self_p->config.bias_precalculated = 0; // set it one to retain the biasing, recalculate otherwise
  self_p->config.data_smoothON = 0;

  //  LINE: 762 Bias: A -1832 -510 16370 | G -7 35 5
  //  LINE: 770 Bias: A -1022 -634 16464 | G -4 32 8
  //  LINE: 770 Bias: A -1384 -620 16448 | G -8 32 8
  //
  std_printf(FSTR("\r\n"
  "LINE: %d : \r\n"
  "self_p->config._internal._sampleRateDiv  %d\r\n"
  "self_p->config.config.sampleRate         %d\r\n"
  "self_p->config.config.filterLevel        %d\r\n"
  "self_p->config._internal._gFSR           %d\r\n"
  "self_p->config._internal._aFSR           %d\r\n"
  "self_p->config._internal._samplePeriod   %d\r\n"
  "self_p->config._internal.accelToG        %f\r\n"
  "self_p->config._internal.gyroToRad       %f\r\n"

)
, __LINE__
, self_p->config._internal._sampleRateDiv
, self_p->config.config.sampleRate
, self_p->config.config.filterLevel
, self_p->config._internal._gFSR
, self_p->config._internal._aFSR
, self_p->config._internal._samplePeriod
, self_p->config._internal.accelToG
, self_p->config._internal.gyroToRad
);

//g_self_p = self_p; // for external libs

return ret;

}

struct mpu6050_basic_driver_t * mpu6050_basic_get_driver()//struct mpu6050_basic_driver_t *self_p)
{
  //self_p = g_self_p; //expected to get this pointer for external libraries
  return (g_self_p);
}

static int transport_i2c_start(struct mpu6050_basic_driver_t *self_p)
{
  int res;

  struct mpu6050_basic_transport_i2c_t *transport_p;

  transport_p = (struct mpu6050_basic_transport_i2c_t *)self_p->transport_p;

  deep_debug_log(FSTR("\r\n"
  "LINE: %d : transport_p->i2c_address 0x%02x.\r\n"), __LINE__, transport_p->i2c_address);

  if (transport_p->i2c_address == MPU6050_BASIC_I2C_ADDRESS_AUTOMATIC)
  {
    #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
    res = i2c_scan(transport_p->i2c_p, MPU6050_BASIC_I2C_ADDRESS_0);
    #else
    res = i2c_soft_scan(transport_p->i2c_p, MPU6050_BASIC_I2C_ADDRESS_0);
    #endif
    if (res == 1)
    {
      transport_p->i2c_address = MPU6050_BASIC_I2C_ADDRESS_0;
    }
    else
    {
      #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
      res = i2c_scan(transport_p->i2c_p, MPU6050_BASIC_I2C_ADDRESS_1);
      #else
      res = i2c_soft_scan(transport_p->i2c_p, MPU6050_BASIC_I2C_ADDRESS_1);
      #endif
      if (res == 1)
      {
        transport_p->i2c_address = MPU6050_BASIC_I2C_ADDRESS_1;
      }
    }
  }
  else
  {
    #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
    res = i2c_scan(transport_p->i2c_p, transport_p->i2c_address);
    #else
    res = i2c_soft_scan(transport_p->i2c_p, transport_p->i2c_address);
    #endif
    deep_debug_log(FSTR("\r\n"
    "LINE: %d : i2c_scan %d.\r\n"), __LINE__, res);

  }

  if (res == 1)
  {
    DLOG(INFO,
      "Found device with I2C address 0x%02x.\r\n",
      transport_p->i2c_address
    );
    res = 0;
    deep_debug_log(FSTR("\r\n"
    "LINE: %d : i2c_scan return %d.\r\n"), __LINE__, res);
  }
  else if (res == 0)
  {
    res = -ENODEV;
  }

  if (res != 0)
  {
    DLOG(INFO,
      "I2C transport start failed with %d.\r\n",
      res

    );
  }

  return (res);
}

static int transport_i2c_read(
  struct mpu6050_basic_driver_t *self_p,
  uint8_t address,
  uint8_t *buf_p,
  size_t size
)
{
  struct mpu6050_basic_transport_i2c_t *transport_p;
  ssize_t res;

  transport_p = (struct mpu6050_basic_transport_i2c_t *)self_p->transport_p;

  #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
  res = i2c_write(
    #else
    res = i2c_soft_write(
      #endif

      transport_p->i2c_p,
      transport_p->i2c_address,
      &address,
      sizeof(address) // size ??
    );

    //i2c_stop(transport_p->i2c_p);
    //i2c_start(transport_p->i2c_p);

    deep_debug_log(FSTR("\r\n"
    "LINE: %d : i2c_write return %d.\r\n"), __LINE__, res);

    if (res != sizeof(address))
    {
      DLOG(ERROR,
        "I2C address write failed with %d.\r\n",
        res
      );
      return (-EIO);
    }

    deep_debug_log(FSTR("\r\n"
    "LINE: %d : check return %d size %d.\r\n"), __LINE__, res, size);

    #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
    res = i2c_read(
      #else
      res = i2c_soft_read(
        #endif
        transport_p->i2c_p,
        transport_p->i2c_address,
        buf_p,
        size
      );

      deep_debug_log(FSTR("\r\n"
      "LINE: %d : i2c_read return %d size %d data 0x%02x.\r\n"), __LINE__, res, size, buf_p[0]);

      if (res != size)
      {
        DLOG(ERROR,
          "I2C read failed with %d.\r\n",
          res
        );
        return (-EIO);
      }

      return (0);
    }

    static int transport_i2c_write(
      struct mpu6050_basic_driver_t *self_p,
      uint8_t address,
      uint8_t data
    )
    {
      struct mpu6050_basic_transport_i2c_t *transport_p;
      ssize_t res;
      uint8_t buf[2];

      transport_p = (struct mpu6050_basic_transport_i2c_t *)self_p->transport_p;

      buf[0] = address;
      buf[1] = data;

      #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
      res = i2c_write(
        #else
        res = i2c_soft_write(
          #endif
          transport_p->i2c_p,
          transport_p->i2c_address,
          &buf[0],
          sizeof(buf)
        );

        deep_debug_log(FSTR("\r\n"
        "LINE: %d res %d.\r\n"), __LINE__, res);


        if (res != sizeof(buf))
        {
          DLOG(ERROR,
            "I2C write failed with %d.\r\n",
            res
          );
          return (-EIO);
        }

        return (0);
      }

      struct mpu6050_basic_transport_protocol_t transport_i2c_protocol =
      {
        .start = transport_i2c_start,
        .read = transport_i2c_read,
        .write = transport_i2c_write
      };


      int mpu6050_basic_module_init()
      {

        #if CONFIG_BMP280_DEBUG_LOG_MASK > -1
        log_module_init();
        #endif

        return (0);
      }

      int mpu6050_basic_start(struct mpu6050_basic_driver_t *self_p, struct sMPUDATA_t *data_p)
      {
        ASSERTN(self_p != NULL, EINVAL);

        int res;
        uint8_t buf[24];
        int i;
        uint8_t chip_id;
        signed char gyro_orientation[9] = {1, 0, 0, 0,1, 0, 0, 0, 1};


        deep_debug_log(FSTR("\r\n"
        "LINE: %d.\r\n"), __LINE__);


        res = self_p->transport_p->protocol_p->start(self_p);

        if (res != 0)
        {
          return (res);
        }

        g_self_p = self_p; // for external libs
        ASSERTN(g_self_p != NULL, EINVAL);

        deep_debug_log(FSTR("\r\n"
        "LINE: %d.\r\n"), __LINE__);

        //Read chip id
        // NOT WORKING

        res = self_p->transport_p->protocol_p->read(
          self_p,
          MPU6050_BASIC_REG_WHO_AM_I,
          &chip_id,
          sizeof(chip_id)
        );

        if (res != 0)
        {
          DLOG(WARNING, "Failed to read chip id with %d.\r\n", res);
          return (res);
        }

        DLOG(INFO, "Chip id: 0x%02x.\r\n", chip_id);
        deep_debug_log(FSTR("\r\n"
        "LINE: %d ID 0x%02x.\r\n"), __LINE__, chip_id);


        #if (CONFIG_MPU6050_DMP_ENABLE > 0)

        res = mpu_init();

        //while(1);
        if (res != 0)
        {
          DLOG(WARNING, "Failed mpu_init %d.\r\n", res);
          deep_debug_log(FSTR("mpu_init %d %d"), __LINE__, res);

          return (res);
        }


        res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

        if (res != 0)
        {
          DLOG(WARNING, "Failed mpu_set_sensors %d.\r\n", res);
          return (res);
        }


        /* Push both gyro and accel data into the FIFO. */
        res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

        if (res != 0)
        {
          DLOG(WARNING, "Failed mpu_configure_fifo %d.\r\n", res);
          return (res);
        }


        res = mpu_set_sample_rate(CONFIG_DEFAULT_MPU_HZ);

        if (res != 0)
        {
          DLOG(WARNING, "Failed mpu_set_sample_rate %d.\r\n", res);
          return (res);
        }


        res = dmp_load_motion_driver_firmware();

        if (res != 0)
        {
          DLOG(WARNING, "Failed dmp_load_motion_driver_firmware %d.\r\n", res);
          return (res);
        }


        res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));

        if (res != 0)
        {
          DLOG(WARNING, "Failed dmp_set_orientation %d.\r\n", res);
          return (res);
        }


        res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT  | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);

        if (res != 0)
        {
          DLOG(WARNING, "Failed dmp_enable_feature %d.\r\n", res);
          return (res);
        }


        res = dmp_set_fifo_rate(CONFIG_DEFAULT_MPU_HZ);

        if (res != 0)
        {
          DLOG(WARNING, "Failed dmp_set_fifo_rate %d.\r\n", res);
          return (res);
        }


        res = mpu_set_dmp_state(1);

        if (res != 0)
        {
          DLOG(WARNING, "Failed mpu_set_dmp_state %d.\r\n", res);
          return (res);
        }


        // mpu.initialize();
        // mpu.dmpInitialize();
        //
        // // This need to be setup individually
        // mpu.setXGyroOffset(220);
        // mpu.setYGyroOffset(76);
        // mpu.setZGyroOffset(-85);
        // mpu.setZAccelOffset(1788);
        //
        // mpu.setDMPEnabled(true);

        return res;

        #endif


        // Set Power mode
        //

        res = self_p->transport_p->protocol_p->write(
          self_p,
          MPU6050_BASIC_REG_PWR_MGMT_1,
          0
        );

        if (res != 0)
        {
          DLOG(WARNING, "Failed to set power mode with %d.\r\n", res);
          return (res);
        }

        struct mpu6050_basic_setting settings[12] =
        {
          { 25,  self_p->config._internal._sampleRateDiv , -1}, 	//  sample rate divider: sample rate = mstrClock / (1 +  divider)
          { 26,  self_p->config.config.filterLevel			, -2},  //  DLPF set.  (0 = 8kHz master clock else 1kHz master clock)
          { 27,  self_p->config._internal._gFSR					, -3},  //  gyro full scale range
          { 28,  self_p->config._internal._aFSR					, -4},  //  accel full scale range
          { 31,  0b00000000			, -5},	//  no motion detect
          { 35,  0b00000000			, -6},	//  no FIFO
          { 36,  0b00000000			, -7},	//  no mstr I2C
          { 55,  0b01110000			, -8},	//	configure interrupt  -- on when data ready, off on data read
          { 56,  0b00000001			, -9},	//	interrupt on
          { 106, 0b00000000		  , -10},	//  no silly stuff
          { 107, 0b00000001			, -11},	//  no sleep and clock off gyro_X
          { 108, 0b00000000			, -12}   //  no goofball sleep mode
        };

        for (int i = 0 ; i < 12 ; i++)
        {

          res = self_p->transport_p->protocol_p->write(
            self_p,
            settings[i].address,
            settings[i].data
          );

          if (res != 0)
          {
            DLOG(WARNING, "Failed: %d with %d.\r\n", settings[i].errmpu, res);
            return (res);
          }
        }

        res = mpu6050_basic_motion_agzero(self_p, data_p);

        if (res != 0)
        {
          DLOG(WARNING, "Failed: to set biasing %d.\r\n", res);
          return (res);
        }

        return (res);

      }

      int mpu6050_basic_stop(struct mpu6050_basic_driver_t *self_p)
      {
        ASSERTN(self_p != NULL, EINVAL);

        //TODO

        return (-ENOSYS);
      }

      int mpu6050_basic_read(
        struct mpu6050_basic_driver_t *self_p,
        struct sMPUDATA_t *data_p
      )
      {
        ASSERTN(self_p != NULL, EINVAL);

        int res;

        res = mpu6050_basic_read_mpu(self_p, data_p);

        return (res);
      }

      int mpu6050_basic_read_mpu(
        struct mpu6050_basic_driver_t *self_p,
        struct sMPUDATA_t *data_p
      )
      {
        ASSERTN(self_p != NULL, EINVAL);

        #define SMOOTH_SIZE (10)
        int res;
        int attempt;

#       if (CONFIG_MPU6050_DMP_ENABLE > 0)

        res = 0;

        // signed char   gyro_orientation[9] = {1, 0, 0, 0,1, 0, 0, 0, 1};
        // float         accel_g[3], accel_p[3] = {0}, accel_final[3];
        // float         q[4], Pitch, Roll,Yaw;
        //
        // int16_t       gyro[3], accel[3], accel_ave[3], accel_res[2][3]={0}, disp_show[3], vel_show[3];
        // long int      vel[2][3]={0}, disp[2][3] = {0};
        // unsigned long timestamp, time_pre;
        // short sensors = INV_XYZ_GYRO| INV_XYZ_ACCEL | INV_WXYZ_QUAT;
        // unsigned char more;
        // long          quat[4];
        //
        // unsigned long time1,time2;
        //
        // /* debug */
        // int16_t       accel_show[3],i;
        //
        // accel_g[0] = 0;
        // accel_g[1] = 0;
        // accel_g[2] = 0.978833;

#       else

        static int smooth_index = 0;
        static struct sMPUDATA_t mpusmoothdatabuf[SMOOTH_SIZE];
        struct sMPUDATA_t mpusmoothdata;
        uint8_t buf[15];

        mpusmoothdata.AcX = 0 ;
        mpusmoothdata.AcY = 0 ;
        mpusmoothdata.AcZ = 0 ;
        mpusmoothdata.GyX = 0 ;
        mpusmoothdata.GyY = 0 ;
        mpusmoothdata.GyZ = 0 ;

        /* Read the pressure and temperature. */
        res = self_p->transport_p->protocol_p->read(self_p,
          MPU6050_BASIC_REG_DATA_RDY_INT_STATUS,
          &buf[0],
          sizeof(buf)
        );

        if (res != 0 )
        {
          if (res != 0)
          {
            DLOG(WARNING, "Failed: to read sensor while data read %d.\r\n", res);
            return (res);
          }

        }

        if(0x01 == (0x01 & buf[0])) // Data ready //TODO: enum / Macro
        {

          res = 0;
          //struct time_t uptime_p;
          //sys_uptime(&uptime_p);
          //data_p->timestamp = time_micros();//uptime_p.seconds;
          //data_p->timestamp = uptime_p.nanoseconds;

          // data_p->AcX = ((float)(int16_t)( buf[1] << 8 | buf[2] )) * self_p->config._internal.accelToG + self_p->config.bias.AcX;
          // data_p->AcY = ((float)(int16_t)( buf[3] << 8 | buf[4] )) * self_p->config._internal.accelToG + self_p->config.bias.AcY;
          // data_p->AcZ = ((float)(int16_t)( buf[5] << 8 | buf[6] )) * self_p->config._internal.accelToG + self_p->config.bias.AcZ;
          // data_p->Tmp = temp_C(( (int16_t) (buf[7] << 8 | buf[8]) ));
          // data_p->GyX = ((float)(int16_t)( buf[9] << 8 | buf[10])) * self_p->config._internal.gyroToRad + self_p->config.bias.GyX;
          // data_p->GyY = ((float)(int16_t)(buf[11] << 8 | buf[12])) * self_p->config._internal.gyroToRad + self_p->config.bias.GyY;
          // data_p->GyZ = ((float)(int16_t)(buf[13] << 8 | buf[14])) * self_p->config._internal.gyroToRad + self_p->config.bias.GyZ;

          data_p->AcX = ((int16_t)( buf[1] << 8 | buf[2] )) - (int16_t)self_p->config.bias.AcX;
          data_p->AcY = ((int16_t)( buf[3] << 8 | buf[4] )) - (int16_t)self_p->config.bias.AcY;
          data_p->AcZ = ((int16_t)( buf[5] << 8 | buf[6] )) - (int16_t)self_p->config.bias.AcZ;
          data_p->Tmp = ((int16_t)( buf[7] << 8 | buf[8] ));
          data_p->GyX = ((int16_t)( buf[9] << 8 | buf[10])) - (int16_t)self_p->config.bias.GyX;
          data_p->GyY = ((int16_t)(buf[11] << 8 | buf[12])) - (int16_t)self_p->config.bias.GyY;
          data_p->GyZ = ((int16_t)(buf[13] << 8 | buf[14])) - (int16_t)self_p->config.bias.GyZ;

          #define NOISE_SUPP_ACC_MASK (0b1111111111111111)
          #define NOISE_SUPP_GYR_MASK (0b1111111111111111)
          data_p->AcX = ( (data_p->AcX & NOISE_SUPP_ACC_MASK) );
          data_p->AcY = ( (data_p->AcY & NOISE_SUPP_ACC_MASK) );
          data_p->AcZ = ( (data_p->AcZ & NOISE_SUPP_ACC_MASK) );
          //data_p->Tmp = ((int16_t)( buf[7] << 8 | buf[8] ));
          data_p->GyX = ( (data_p->GyX & NOISE_SUPP_GYR_MASK) );
          data_p->GyY = ( (data_p->GyY & NOISE_SUPP_GYR_MASK) );
          data_p->GyZ = ( (data_p->GyZ & NOISE_SUPP_GYR_MASK) );

          mpusmoothdatabuf[smooth_index++ % SMOOTH_SIZE] = *data_p;

          for(int i = 0 ; i < SMOOTH_SIZE ; i++)
          {

            mpusmoothdata.AcX += mpusmoothdatabuf[i].AcX;
            mpusmoothdata.AcY += mpusmoothdatabuf[i].AcY;
            mpusmoothdata.AcZ += mpusmoothdatabuf[i].AcZ;
            mpusmoothdata.GyX += mpusmoothdatabuf[i].GyX;
            mpusmoothdata.GyY += mpusmoothdatabuf[i].GyY;
            mpusmoothdata.GyZ += mpusmoothdatabuf[i].GyZ;

          }

          mpusmoothdata.AcX /= SMOOTH_SIZE;
          mpusmoothdata.AcY /= SMOOTH_SIZE;
          mpusmoothdata.AcZ /= SMOOTH_SIZE;
          mpusmoothdata.GyX /= SMOOTH_SIZE;
          mpusmoothdata.GyY /= SMOOTH_SIZE;
          mpusmoothdata.GyZ /= SMOOTH_SIZE;

          //smooth_index = 0;

          if(1 == self_p->config.data_smoothON)
          {
            data_p->AcX = mpusmoothdata.AcX;
            data_p->AcY = mpusmoothdata.AcY;
            data_p->AcZ = mpusmoothdata.AcZ;
            //data_p->Tmp = ((int16_t)( buf[7] << 8 | buf[8] ));
            data_p->GyX = mpusmoothdata.GyX;
            data_p->GyY = mpusmoothdata.GyY;
            data_p->GyZ = mpusmoothdata.GyZ;
          }
          //std_printf(FSTR("\r\n"
          //                  "LINE: %d Ax %d.\r\n"), __LINE__, data_p->AcX );
        }
        else
        {
          // invalid data
        }

#       endif

        return (res);
      }

      int mpu6050_basic_transport_i2c_init(
        struct mpu6050_basic_transport_i2c_t *self_p,
        #if (CONFIG_MPU6050_BASIC_USE_HARD_I2C>-1)
        struct i2c_driver_t *i2c_p,
        #else
        struct i2c_soft_driver_t *i2c_p,
        #endif

        int i2c_address
      )
      {
        ASSERTN(self_p != NULL, EINVAL);
        ASSERTN(i2c_p != NULL, EINVAL);
        ASSERTN((i2c_address == MPU6050_BASIC_I2C_ADDRESS_0)
        || (i2c_address == MPU6050_BASIC_I2C_ADDRESS_1)
        || (i2c_address == MPU6050_BASIC_I2C_ADDRESS_AUTOMATIC), EINVAL);

        #if CONFIG_I2C == 1
        self_p->base.protocol_p = &transport_i2c_protocol;
        self_p->i2c_p = i2c_p;
        self_p->i2c_address = i2c_address;

        return (0);
        #else
        return (-ENOSYS);
        #endif
      }

      // must be called every config.sampleRate duration uS
      // If not being called this should msg for timeout warnings
      int mpu6050_motion_calc(struct mpu6050_basic_driver_t *self_p, struct sMPUDATA_t *data_p, struct Vec3 *ypr_p, const uint32_t dt_us)
      {

        //static Madgwick Madgwick;
        //struct Quat AttitudeEstimateQuat;

        // MadgwickAHRSupdateIMU(&AttitudeEstimateQuat
        //  , ((float)data_p->GyX)*self_p->config._internal.gyroToRad, ((float)data_p->GyY)*self_p->config._internal.gyroToRad, ((float)data_p->GyZ)*self_p->config._internal.gyroToRad
        //   , ((float)data_p->AcX)*self_p->config._internal.accelToG, ((float)data_p->AcY)*self_p->config._internal.accelToG, ((float)-data_p->AcZ)*self_p->config._internal.accelToG);

        //toEulerAngle(&AttitudeEstimateQuat, ypr_p);

        //ypr_p->x = RAD_TO_DEG*(ypr_p->x);
        //ypr_p->y = RAD_TO_DEG*(ypr_p->y);
        //ypr_p->z = RAD_TO_DEG*(ypr_p->z);

        //return (0);
#       if (CONFIG_MPU6050_DMP_ENABLE > 0)

        //signed char   gyro_orientation[9] = {1, 0, 0, 0,1, 0, 0, 0, 1};
        static float         accel_g[3], accel_p[3], accel_final[3];
        static float         q[4], Pitch, Roll,Yaw;

        static int16_t       gyro[3], accel[3], accel_ave[3], accel_res[2][3], disp_show[3], vel_show[3];
        static long int      vel[2][3], disp[2][3];
        static unsigned long timestamp, time_pre;
        static short sensors = INV_XYZ_GYRO| INV_XYZ_ACCEL | INV_WXYZ_QUAT;
        static unsigned char more;
        static long          quat[4];

        static unsigned long time1,time2;

        /* debug */
        static int16_t       accel_show[3],i;

        // accel_g[0] = 0;
        // accel_g[1] = 0;
        // accel_g[2] = 0.978833;

        Pitch = 0;
        Roll = 0;
        Yaw = 0;


        dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
//        printf("%ld\n",timestamp);
        if((sensors & INV_WXYZ_QUAT))
        {
           /* DMP */
           q[0]=quat[0] / 1073741824.0f;
           q[1]=quat[1] / 1073741824.0f;
           q[2]=quat[2] / 1073741824.0f;
           q[3]=quat[3] / 1073741824.0f;

           Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]) *57.3; // pitch
           Roll = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)*57.3; // roll
           Yaw = 	atan2(2*(q[1]*q[2] + q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3])*57.3 ;		//

           ypr_p->x = Yaw;//(ypr_p->x);
           ypr_p->y = Pitch;//(ypr_p->y);
           ypr_p->z = Roll;//(ypr_p->z);

//			   printf("%3f,%3f,%3f\n",Pitch,Roll, Yaw);
        }


#       else


        static struct Quat AttitudeEstimateQuat = {1.0f, 0.0f, 0.0f, 0.0f};

        static struct Vec3 correction_Body, correction_World;
        static struct Vec3 Accel_Body, Accel_World;
        static struct Vec3 GyroVec;
        struct Quat incrementalRotation;
        // Sensor is placed upside down approx
        const struct Vec3 VERTICAL = Vector(0.0f, 0.0f, 1.0f);
        // vertical vector in the World frame

        //std_printf(OSTR("\r\n\r\n"));
        //std_printf(OSTR("Calc Debug: %d: (%d, %d, %d), (%d, %d, %d) "), __LINE__, data_p->AcX, data_p->AcY, -data_p->AcZ, data_p->GyX, data_p->GyY, data_p->GyZ);
        GyroVec    = Vector(
          ((float)data_p->GyX)*self_p->config._internal.gyroToRad,
          ((float)data_p->GyY)*self_p->config._internal.gyroToRad,
          ((float)data_p->GyZ)*self_p->config._internal.gyroToRad
        );
        Accel_Body = Vector(
          ((float)data_p->AcX)*self_p->config._internal.accelToG,
          ((float)data_p->AcY)*self_p->config._internal.accelToG,
          ((float)-data_p->AcZ)*self_p->config._internal.accelToG
        );


        //GyroVec = NormalizeV(GyroVec);
        //Accel_Body = NormalizeV(Accel_Body);

        //GyroVec  = Vector((float)data_p->GyX, (float)data_p->GyY, (float)data_p->GyZ);	// move gyro data to vector structure
        //Accel_Body = Vector((float)data_p->AcX, (float)data_p->AcY, (float)-data_p->AcZ);	// move accel data to vector structure
        // Manipulated z as -z as sensor placement seemed inverted to me
        // It seems there will be edge cases on border angles

        //std_printf(OSTR("Calc Debug: %d: (%f, %f, %f), (%f, %f, %f)\r\n"), __LINE__
        //, GyroVec.x, GyroVec.y, GyroVec.z
        //, Accel_Body.x, Accel_Body.y, Accel_Body.z);

        //GyroVec = NormalizeV(GyroVec);
        //Accel_Body = NormalizeV(Accel_Body);
        //std_printf(OSTR("Calc Debug: %d: (%f, %f, %f), (%f, %f, %f)\r\n"), __LINE__
        //, GyroVec.x, GyroVec.y, GyroVec.z
        //, Accel_Body.x, Accel_Body.y, Accel_Body.z);


        Accel_World = RotateQV(AttitudeEstimateQuat, Accel_Body); // rotate accel from body frame to world frame
        //Accel_World = NormalizeV(Accel_World);

        // std_printf(OSTR("Calc Debug: %d: (%f, %f, %f, %f), (%f, %f, %f) "), __LINE__
        // , AttitudeEstimateQuat.x, AttitudeEstimateQuat.y, AttitudeEstimateQuat.z, AttitudeEstimateQuat.w
        // , Accel_World.x, Accel_World.y, Accel_World.z);

        correction_World = CrossProdVV(Accel_World, VERTICAL); // cross product to determine error
        //correction_World = NormalizeV(correction_World);

        // std_printf(OSTR("Calc Debug: %d: (%f, %f, %f), (%f, %f, %f) "), __LINE__
        // , correction_World.x, correction_World.y, correction_World.z
        // , Accel_World.x, Accel_World.y, Accel_World.z);

        //correction_Body = RotateVQ(correction_World, AttitudeEstimateQuat); // rotate correction vector to body frame
        correction_Body = RotateQV(AttitudeEstimateQuat, correction_World);
        //correction_Body = NormalizeV(correction_Body);

        // std_printf(OSTR("Calc Debug: %d: (%f, %f, %f), (%f, %f, %f) "), __LINE__
        // , correction_World.x, correction_World.y, correction_World.z
        // , correction_Body.x, correction_Body.y, correction_Body.z);

        GyroVec = SumVV(GyroVec, correction_Body);  // add correction vector to gyro data
        //GyroVec = NormalizeV(GyroVec);

        //std_printf(OSTR("Calc Debug: %d: (%f, %f, %f) "), __LINE__
        //, correction_Body.x, correction_Body.y, correction_Body.z);

        incrementalRotation = QuaternionVS(GyroVec, dt_us);//self_p->config._internal._samplePeriod);  // create incremental rotation quat

        incrementalRotation = NormalizeQ(incrementalRotation);

        //std_printf(OSTR("Calc Debug: %d: (%f, %f, %f, %f)\r\n"), __LINE__
        //, incrementalRotation.x, incrementalRotation.y, incrementalRotation.z, incrementalRotation.w);

        AttitudeEstimateQuat = MulQQ(incrementalRotation, AttitudeEstimateQuat);  // quaternion integration (rotation composting through multiplication)

        AttitudeEstimateQuat = NormalizeQ(AttitudeEstimateQuat);
        //std_printf(OSTR("Calc Debug: %d: (%f, %f, %f, %f) "), __LINE__
        //, AttitudeEstimateQuat.x, AttitudeEstimateQuat.y, AttitudeEstimateQuat.z, AttitudeEstimateQuat.w);

        *ypr_p = YawPitchRoll(AttitudeEstimateQuat);

        //NormalizeV(*ypr_p); // Not valid

        ypr_p->x = RAD_TO_DEG*(ypr_p->x);
        ypr_p->y = RAD_TO_DEG*(ypr_p->y);
        ypr_p->z = RAD_TO_DEG*(ypr_p->z);

#       endif

        return (0);
      }

      int mpu6050_basic_motion_agzero(struct mpu6050_basic_driver_t *self_p, struct sMPUDATA_t *data_p)	// Measure and store accel and gyro bias offsets
      {

        int res;

        if(1 == self_p->config.bias_precalculated)
        return (0);

        float sampleCount = 0;

        float sampleTempAX = 0;
        float sampleTempAY = 0;
        float sampleTempAZ = 0;

        float sampleTempGX = 0;
        float sampleTempGY = 0;
        float sampleTempGZ = 0;

#       if (CONFIG_MPU6050_DMP_ENABLE > 0)

#       else

        self_p->config.bias.AcX = 0;
        self_p->config.bias.AcY = 0;
        self_p->config.bias.AcZ = 0;
        self_p->config.bias.GyX = 0;
        self_p->config.bias.GyY = 0;
        self_p->config.bias.GyZ = 0;

#       endif

        while(sampleCount<100) // init time
        {
          sampleCount++;
          res = mpu6050_basic_read_mpu(self_p, data_p);
          if (res != 0)
          {
            DLOG(WARNING, "Failed: to read sensor while warmup %d.\r\n", res);
            return (res);
          }
        }

        sampleCount = 0;
        while(sampleCount<100)
        {

          res = mpu6050_basic_read_mpu(self_p, data_p);

          if (res != 0)
          {
            DLOG(WARNING, "Failed: to read sensor while biasing %d.\r\n", res);
            return (res);
          }

#       if (CONFIG_MPU6050_DMP_ENABLE > 0)

#       else

          sampleTempAX += (float)data_p->AcX;
          sampleTempAY += (float)data_p->AcY;
          sampleTempAZ += (float)data_p->AcZ;

          sampleTempGX += (float)data_p->GyX;
          sampleTempGY += (float)data_p->GyY;
          sampleTempGZ += (float)data_p->GyZ;

#         endif

          sampleCount += 1;

          //thrd_sleep_us(self_p->config._internal._samplePeriod);
        }

        sampleTempAX = sampleTempAX;
        sampleTempAY = sampleTempAY;
        sampleTempAZ = sampleTempAZ;

        sampleTempGX = sampleTempGX;
        sampleTempGY = sampleTempGY;
        sampleTempGZ = sampleTempGZ;

        // if(sampleTempAX > (.75f * sampleCount)){sampleTempAX -= sampleCount;}
        // if(sampleTempAY > (.75f * sampleCount)){sampleTempAY -= sampleCount;}
        // if(sampleTempAZ > (.75f * sampleCount)){sampleTempAZ -= sampleCount;}
        //
        // if(sampleTempAX < (-.75f * sampleCount)){sampleTempAX += sampleCount;}
        // if(sampleTempAY < (-.75f * sampleCount)){sampleTempAY += sampleCount;}
        // if(sampleTempAZ < (-.75f * sampleCount)){sampleTempAZ += sampleCount;}

#       if (CONFIG_MPU6050_DMP_ENABLE > 0)

#       else

        self_p->config.bias.AcX = (int16_t)(sampleTempAX / sampleCount); // average
        self_p->config.bias.AcY = (int16_t)(sampleTempAY / sampleCount);
        self_p->config.bias.AcZ = (int16_t)(sampleTempAZ / sampleCount);

        self_p->config.bias.GyX = (int16_t)(sampleTempGX / sampleCount); // average
        self_p->config.bias.GyY = (int16_t)(sampleTempGY / sampleCount);
        self_p->config.bias.GyZ = (int16_t)(sampleTempGZ / sampleCount);

#       endif

        // std_printf(FSTR("\r\n"
        // "LINE: %d Bias: "
        // "A %d %d %d"
        // " | G %d %d %d"
        // "\r\n"), __LINE__,
        // data_p->AcX, data_p->AcY, data_p->AcZ,
        // data_p->GyX, data_p->GyY, data_p->GyZ
      //);

      //                  while(1);

      return 0;

    }

    float temp_C(uint16_t tempRaw)	// return sensor temperature
    {
      return((float)(tempRaw) * TEMP_SCALE + TEMP_OFFSET);
    }
