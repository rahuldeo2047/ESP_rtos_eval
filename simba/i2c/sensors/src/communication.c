
#include "simba.h"
#include "communication.h"

#include "wificonfig.h"

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


   start_wifi_station();

   return (void*)(0);

}
