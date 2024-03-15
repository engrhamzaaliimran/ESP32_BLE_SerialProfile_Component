#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_spp.h"


void app_main()
{
    set_generic_access_name("Example_demo");
    set_device_name("Example_demo");
    ble_spp_init();
}
