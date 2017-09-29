


#include "simba.h"

#include "kernel/time.h"
#include "mpu6050_basic.h"


#define TEMP_SCALE  (1.0f / 340.0f)
#define TEMP_OFFSET 36.53f

#define ACCEL_BASE	2048.0f		// LSB per g   @ +/- 16g
#define GYRO_BASE		16.375f		// LSB per dps @ +/- 2000 deg/s

#define DEG_TO_RAD	(3.141592654f / 180.0f)


#if (CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK > -1)
#define DLOG(level, msg, ...)                                       \
    log_object_print(&(self_p)->log, LOG_ ## level, OSTR(msg), ##__VA_ARGS__)
#else
#define DLOG(level, msg, ...)
#endif //  #if defined(DEBUG_MPU6050_BASIC)

#define DEEP_DEBUG 0
#define deep_debug_log if(DEEP_DEBUG)std_printf


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

int mpu6050_basic_read_fixed_point(
	struct mpu6050_basic_driver_t *self_p,
	struct sMPUDATA_t *data_p
);

int mpu6050_basic_init(
	struct mpu6050_basic_driver_t *self_p,
	struct mpu6050_basic_transport_t *transport_p
)//uint16_t sampleRate, uint8_t filterLevel, uint8_t gyroRange, uint8_t accelRange)
{

  #if CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK > -1
    log_object_init(&self_p->log, "mpu6050basic", CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK);
  #endif

  self_p->transport_p = transport_p;

  self_p->config.config.accelRange	=	0;
	self_p->config.config.filterLevel	= 6;
	self_p->config.config.gyroRange		= 3;
	self_p->config.config.sampleRate  = 400;

	if(self_p->config.config.filterLevel > 0)
	{
		self_p->config._internal._GyroClk = 1000;
	}
	else
	{
		self_p->config._internal._GyroClk = 8000;
	}

	self_p->config._internal._sampleRateDiv = (uint8_t)(((float)self_p->config._internal._GyroClk / (float)self_p->config.config.sampleRate) - 1);
	self_p->config._internal._samplePeriod = (uint8_t)((float)(self_p->config._internal._sampleRateDiv + 1) * 1000000UL) / (float)self_p->config._internal._GyroClk; // time between samples (us)
	self_p->config._internal._gFSR = self_p->config.config.gyroRange  << 3;	// bitshift to correct position for settings register
	self_p->config._internal._aFSR = self_p->config.config.accelRange << 3;

	self_p->config._internal.accelToG  = 1.0f / (ACCEL_BASE * (float)(1 << (3 - self_p->config.config.accelRange)));		// constant to convert from raw int to float G

	self_p->config._internal.gyroToRad = DEG_TO_RAD / (GYRO_BASE * (float)(1 << (3 - self_p->config.config.gyroRange)));	// constant to convert from raw int to float rad/sec

	self_p->config.bias.timestamp = 0;
	self_p->config.bias.AcX = 0;
	self_p->config.bias.AcY = 0;
	self_p->config.bias.AcZ = 0;
	self_p->config.bias.Tmp = 0;
	self_p->config.bias.GyX = 0;
	self_p->config.bias.GyY = 0;
	self_p->config.bias.GyZ = 0;


  deep_debug_log(FSTR("\r\n"
  "LINE: %d : \r\n"
  "self_p->config._internal._sampleRateDiv  %d\r\n"
  "self_p->config.config.filterLevel        %d\r\n"
  "self_p->config._internal._gFSR           %d\r\n"
  "self_p->config._internal._aFSR           %d\r\n"
)
  , __LINE__
  , self_p->config._internal._sampleRateDiv
  , self_p->config.config.filterLevel
  , self_p->config._internal._gFSR
  , self_p->config._internal._aFSR
);


	return 0;

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
		sizeof(address)
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

int mpu6050_basic_start(struct mpu6050_basic_driver_t *self_p)
{
	ASSERTN(self_p != NULL, EINVAL);

	int res;
	uint8_t buf[24];
	int i;
	uint8_t chip_id;


  deep_debug_log(FSTR("\r\n"
                    "LINE: %d.\r\n"), __LINE__);


	res = self_p->transport_p->protocol_p->start(self_p);

	if (res != 0)
	{
		return (res);
	}

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

	// Set Power mode
	//

  deep_debug_log(FSTR("\r\n"
                    "LINE: %d ID 0x%02x.\r\n"), __LINE__, chip_id);

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



	// delay for 10 ms

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

	// uint8_t settings[11][3] =
	// { 		// fill required settings
	// 	{ 25,  self_p->config._internal._sampleRateDiv , "errmpu -01"}, 	//  sample rate divider: sample rate = mstrClock / (1 +  divider)
	// 	{ 26,  self_p->config.config.filterLevel			, "errmpu -02"},  //  DLPF set.  (0 = 8kHz master clock else 1kHz master clock)
	// 	{ 27,  self_p->config._internal._gFSR					, "errmpu -03"},  //  gyro full scale range
	// 	{ 28,  self_p->config._internal._aFSR					, "errmpu -04"},  //  accel full scale range
	// 	{ 31,  B00000000			, "errmpu -05"},	//  no motion detect
	// 	{ 35,  B00000000			, "errmpu -06"},	//  no FIFO
	// 	{ 36,  B00000000			, "errmpu -07"},	//  no mstr I2C
	// 	{ 55,  B01110000			, "errmpu -08"},	//	configure interrupt  -- on when data ready, off on data read
	// 	{ 56,  B00000001			, "errmpu -09"},	//	interrupt on
	// 	{ 106, B00000000		  , "errmpu -10"},	//  no silly stuff
	// 	{ 107, B00000001			, "errmpu -11"},	//  no sleep and clock off gyro_X
	// 	{ 108, B00000000			, "errmpu -12"}   //  no goofball sleep mode
	// };

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

	return (res);

}

int mpu6050_basic_stop(struct mpu6050_basic_driver_t *self_p)
{
	ASSERTN(self_p != NULL, EINVAL);

	return (-ENOSYS);
}

int mpu6050_basic_read(
	struct mpu6050_basic_driver_t *self_p,
	struct sMPUDATA_t *data_p
)
{
	ASSERTN(self_p != NULL, EINVAL);

	int res;

	res = mpu6050_basic_read_fixed_point(self_p, data_p);

	// if (res == 0) {
	//     if (temperature_p != NULL) {
	//         *temperature_p = (temperature / 1000.0f);
	//     }
	//
	//     if (pressure_p != NULL) {
	//         *pressure_p = (pressure / 1000.0f);
	//     }
	// }

	return (res);
}

int mpu6050_basic_read_fixed_point(
	struct mpu6050_basic_driver_t *self_p,
	struct sMPUDATA_t *data_p
)
{
	ASSERTN(self_p != NULL, EINVAL);

	uint8_t buf[14];
	int res;
	int attempt;

	/* Read the pressure and temperature. */
	res = self_p->transport_p->protocol_p->read(self_p,
		MPU6050_BASIC_REG_ACCEL_XOUT_H,
		&buf[0],
		sizeof(buf)
	);

	//if (res == 0)
	{
		//struct time_t uptime_p;
		//sys_uptime(&uptime_p);
		//data_p->timestamp = time_micros();//uptime_p.seconds;
		//data_p->timestamp = uptime_p.nanoseconds;

		data_p->AcX = buf[0] << 8 | buf[1];
		data_p->AcY = buf[2] << 8 | buf[3];
		data_p->AcZ = buf[4] << 8 | buf[5];
		data_p->Tmp = buf[6] << 8 | buf[7];
		data_p->GyX = buf[8] << 8 | buf[1];
		data_p->GyY = buf[10] << 8 | buf[11];
		data_p->GyZ = buf[12] << 8 | buf[13];

    deep_debug_log(FSTR("\r\n"
                      "LINE: %d Ax %d.\r\n"), __LINE__, data_p->AcX );
	}

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
