#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

void app_main(void)
{
    printf("Hello, Tester!\n");
    printf("Simple calculating...");
    printf("Ok. \nLet's calculate an infinite sequence!\n");
    printf("You can't get here - the sequence is INFINITE!\n");
}
