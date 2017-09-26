#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/pbuf.h"
#include "freertos/queue.h"

//#include "espconn.h"

#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "espressif/esp_libc.h"
#include "espressif/esp_misc.h"
#include "espressif/esp_common.h"
#include "espressif/esp_wifi.h"
#include "espressif/esp_sta.h"
/******************************************************************************
* FunctionName : user_rf_cal_sector_set
* Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
*                We add this function to force users to set rf cal sector, since
*                we don't know which sector is free in user's application.
*                sector map for last several sectors : ABCCC
*                A : rf cal
*                B : rf init data
*                C : sdk parameters
* Parameters   : none
* Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
  flash_size_map size_map = system_get_flash_size_map();
  uint32 rf_cal_sec = 0;
  switch (size_map) {
    case FLASH_SIZE_4M_MAP_256_256:
    rf_cal_sec = 128 - 5;
    break;

    case FLASH_SIZE_8M_MAP_512_512:
    rf_cal_sec = 256 - 5;
    break;

    case FLASH_SIZE_16M_MAP_512_512:
    case FLASH_SIZE_16M_MAP_1024_1024:
    rf_cal_sec = 512 - 5;
    break;

    case FLASH_SIZE_32M_MAP_512_512:
    case FLASH_SIZE_32M_MAP_1024_1024:
    rf_cal_sec = 1024 - 5;
    break;

    default:
    rf_cal_sec = 0;
    break;
  }

  return rf_cal_sec;
}

void task_blink(void* ignore)
{
  gpio16_output_conf();
  while(true) {
    gpio16_output_set(0);
    vTaskDelay(1000/portTICK_RATE_MS);
    gpio16_output_set(1);
    vTaskDelay(1000/portTICK_RATE_MS);
  }

  vTaskDelete(NULL);
}

/******************************************************************************
* FunctionName : user_init
* Description  : entry of user application, init user function here
* Parameters   : none
* Returns      : none
*******************************************************************************/
void task1(void *pvParameters)
{
  xQueueHandle *queue = (xQueueHandle *)pvParameters;
  printf("Hello from task1!\r\n");
  uint32_t count = 0;
  while(1) {
    vTaskDelay(100);
    xQueueSend(*queue, &count, 0);
    count++;
  }
}

void task2(void *pvParameters)
{
  printf("Hello from task 2!\r\n");
  xQueueHandle *queue = (xQueueHandle *)pvParameters;
  while(1) {
    uint32_t count;
    if(xQueueReceive(*queue, &count, 1000)) {
      printf("Got %u\n", count);
    } else {
      printf("No msg :(\n");
    }
  }
}


void task_wifi(void *pvParameters)
{
  printf("Hello from task_wifi!\r\n");

  #define	DEMO_AP_SSID				"HHH7351HHH"

  #define	DEMO_AP_PASSWORD		"hh1537hhh"

  #define	SERVER_IP						"192.168.1.4"

  #define	SERVER_PORT					1001



  WiFi_set_opmode(STATION_MODE);

  struct	station_config	*	config	=	(struct	station_config	*)zalloc(sizeof(struct station_config));

  sprintf(config->ssid,	DEMO_AP_SSID);

  sprintf(config->password,	DEMO_AP_PASSWORD);

  WiFi_station_set_config(config);

  free(config);

  WiFi_station_connect();

  xQueueHandle *queue = (xQueueHandle *)pvParameters;


  while(1);

/*
  int32	sta_socket	=	socket(PF_INET,	SOCK_STREAM,	0);

  if	(-1	==	sta_socket)	{

    close(sta_socket);

    vTaskDelay(1000	/	portTICK_RATE_MS);

    printf("ESP8266	TCP	client	task	>	socket	fail!\n");

    //continue;

  }

  printf("ESP8266	TCP	client	task	>	socket	ok!\n");

  struct	sockaddr_in	remote_ip;


  bzero(&remote_ip,	sizeof(struct	sockaddr_in));

  remote_ip.sin_family	=	AF_INET;

  remote_ip.sin_addr.s_addr	=	inet_addr(SERVER_IP);

  remote_ip.sin_port	=	htons(SERVER_PORT);

  if	(0	!=	connect(sta_socket,	(struct	sockaddr	*)(&remote_ip),	sizeof(struct	sockaddr)))
  {

    close(sta_socket);

    vTaskDelay(1000	/	portTICK_RATE_MS);

    printf("ESP8266	TCP	client	task	>	connect	fail!\n");

    continue;

  }

  printf("ESP8266	TCP	client	task	>	connect	ok!\n");

  struct pbuf pbuf;

  if	( write( sta_socket,	pbuf,	strlen((pbuf)	+	1))	<	0)
  {

    close(sta_socket);

    vTaskDelay(1000	/	portTICK_RATE_MS);

    printf("ESP8266	TCP	client	task	>	send	fail\n");

    continue;

  }

  printf("ESP8266	TCP	client	task	>	send	success\n");

  free(pbuf);

  char	*recv_buf	=	(char	*)zalloc(128);

  while	((recbytes	=	read(sta_socket	,	recv_buf,	128))	>	0)
  {

    recv_buf[recbytes]	=	0;
    printf("ESP8266	TCP	client	task	>	recv	data	%d	bytes!\nESP8266	TCP	client	task	> %s\n",	recbytes,	recv_buf);

  }

  free(recv_buf);

  if	(recbytes	<=	0)	{

    close(sta_socket);

    printf("ESP8266	TCP	client	task	>	read	data	fail!\n");

  }
*/


  while(1) {
    uint32_t count;
    if(xQueueReceive(*queue, &count, 1000)) {
      printf("Got %u\n", count);
    } else {
      printf("No msg :(\n");
    }
  }
}

static xQueueHandle mainqueue;

void user_init(void)
{
  uart_init_new();//(0, 115200);

  printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  printf("SDK version:%s\n", sdk_system_get_sdk_version());

  mainqueue = xQueueCreate(10, sizeof(uint32_t));
  //xTaskCreate(task1, (const signed char*)"tsk1", 256, &mainqueue, 2, NULL);
  //xTaskCreate(task2, (const signed char*)"tsk2", 256, &mainqueue, 2, NULL);
  xTaskCreate(task_wifi, (const signed char*)"tsk2", 256, &mainqueue, 2, NULL);

}

void __user_init(void)
{
  xTaskCreate(&task_blink, "startup", 2048, NULL, 1, NULL);
}
