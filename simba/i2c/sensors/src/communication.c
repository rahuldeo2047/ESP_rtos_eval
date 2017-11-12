
#include "simba.h"
#include "communication.h"

#include "wificonfig.h"

#include "common.h"

#include "imu_basic.h"
#include "mpu6050_basic.h"


static int start_wifi_station(void)
{
  enum esp_wifi_station_status_t status;

  BTASSERT(esp_wifi_set_op_mode(esp_wifi_op_mode_station_t) == 0);
  BTASSERT(esp_wifi_station_init(STRINGIFY(SSID),
  STRINGIFY(PASSWORD),
  NULL,
  NULL) == 0);
  BTASSERT(esp_wifi_station_connect() == 0);

  esp_wifi_print(sys_get_stdout());

  /* Wait until connected. */
  status = esp_wifi_station_status_idle_t;

  while (status != esp_wifi_station_status_got_ip_t) {
    status = esp_wifi_station_get_status();
    std_printf(FSTR("status: %d\r\n"), status);
    thrd_sleep(1);
  }

  esp_wifi_print(sys_get_stdout());

  //BTASSERT(esp_wifi_set_op_mode(esp_wifi_op_mode_null_t) == 0);

  return (0);
}

void *comm_thrd(void *arg_p)
{

  // SETUP

  int res ;
  struct socket_t tcp;
  struct inet_addr_t local_addr, remote_addr;
  struct imu_thrd_data_t imudata;

  struct bus_info_t * bus_info = (struct bus_info_t *)arg_p;

  memset((void*)&imudata, 0, sizeof(imudata));

  start_wifi_station();

  res = socket_module_init();
  res = res;

  // get these from wificonfig.h

  /* Set the local and remote addresses. */
  inet_aton("192.168.1.103", &local_addr.ip);
  local_addr.port = 6000;
  inet_aton("192.168.1.4", &remote_addr.ip);
  remote_addr.port = 5000;

  /* Initialize the socket and connect to the server. */
  socket_open_tcp(&tcp);
  socket_bind(&tcp, &local_addr);
  socket_connect(&tcp, &remote_addr);

  // tx rx form bus and ros

  while(true)
  {
    if(queue_read((bus_info->queue), &imudata, sizeof(imudata)) > 0)// sizeof(imudata))
    {
      /* Send the data. */
      socket_write(&tcp, &imudata, sizeof(imudata));
      memset((void*)&imudata, 0, sizeof(imudata));

    }
  }

  return (void*)(0);

}
