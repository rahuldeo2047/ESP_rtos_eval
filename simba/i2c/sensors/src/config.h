#ifndef __CONFIG_H__
#define __CONFIG_H__

//#define CONFIG_MINIMAL_SYSTEM                               1
//#define CONFIG_PIN                                          1
#define CONFIG_START_CONSOLE_UART_BAUDRATE                    74800
#define CONFIG_EXTI                                           1

#define CONFIG_MPU6050_BASIC_DEBUG_LOG_MASK                   1
#define CONFIG_MPU6050_BASIC_USE_HARD_I2C                     1 //(-1)

#define CONFIG_MODULE_INIT_PWM_SOFT                           0

#define CONFIG_SYSTEM_TICK_FREQUENCY                          100   // Hz
#define CONFIG_SYSTEM_TICK_SOFTWARE                           1
#define CONFIG_SYSTEM_TICK_SOFTWARE_DURATION_MS               (1000/(CONFIG_SYSTEM_TICK_FREQUENCY))


// MPU specific
#define CONFIG_MOTION_DRIVER_TARGET_ESP_SIMBA                 1
// FreeRTOS
#define configTICK_RATE_HZ			                              ( ( portTickType ) CONFIG_SYSTEM_TICK_FREQUENCY )


//#define CONFIG_EEPROM_SOFT_SEMAPHORE                          0
//#define CONFIG_FLASH_DEVICE_SEMAPHORE                         0
//#define CONFIG_UART_SOFT                                      0

#endif
