
#include "simba.h"
#include "communication.h"

#include "wificonfig.h"

#include "common.h"

#include "imu_basic.h"
#include "mpu6050_basic.h"

#include "/home/rahuldeo/ATOM/ATOM_DRONE/ESP8266_D1/src/data2.h"


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

  thrd_set_name(comm_thrd_get_thrd_name());

  // SETUP

  int res, lastsocketconnstatus ;
  struct socket_t tcp;
  struct inet_addr_t local_addr, remote_addr;
  struct imu_thrd_data_t imudata;
  debug_data debug_data;
  sGENERICSETPOINTS_t setpoints;

  struct inet_if_ip_info_t ip_info;

  struct bus_info_t * bus_info = (struct bus_info_t *)arg_p;

  memset((void*)&imudata, 0, sizeof(imudata));

  start_wifi_station();
  
  res = esp_wifi_station_get_ip_info(&ip_info);

  res = socket_module_init();
  res = res;

  // get these from wificonfig.h

  /* Set the local and remote addresses. */
  local_addr.ip = ip_info.address;
  //inet_aton("192.168.1.103", &local_addr.ip);
  local_addr.port = 10000;
  inet_aton("192.168.1.4", &remote_addr.ip);
  remote_addr.port = 11511;

  /* Initialize the socket and connect to the server. */
  res = socket_open_tcp(&tcp);
  res = socket_bind(&tcp, &local_addr);
  lastsocketconnstatus = socket_connect(&tcp, &remote_addr); // need to be inside the while loop
  if(0!=lastsocketconnstatus)
  {
    // TODO: log PROPERLY
    std_printf(OSTR("--------- socket connect error ---------"));
    while(true);
  }
  // tx rx form bus and ros


  while(true)
  {
    //thrd_sleep_ms(100);
    res = socket_connect(&tcp, &remote_addr); // need to be inside the while loop
    if(0!=res && lastsocketconnstatus != res)
    {
      // TODO: log PROPERLY
      std_printf(OSTR("--------- socket connect error ---------\r\n"));
      //while(true);
      thrd_sleep_ms(500);
      continue;
    }

    if(queue_read((bus_info->queue), &imudata, sizeof(imudata)) > 0)// sizeof(imudata))
    {
      debug_data.mpuRAW = imudata.mpudata;
      /* Send the data. */
      res = socket_write(&tcp, &debug_data, sizeof(debug_data));
      memset((void*)&debug_data, 0, sizeof(debug_data));

      //ssize_t len;
      //len = socket_size(&tcp);

      //if(len >= sizeof(setpoints))
      //  res = socket_read(&tcp, &setpoints, sizeof(setpoints));

      if(0>res)
      {
        // error
      }


    }
  }

  return (void*)(0);

}
