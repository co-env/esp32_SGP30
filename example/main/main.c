/**
 * @file main.c
 * 
 * @author Renato Freitas
 * 
 * @brief SGP30 Air Quality Sensor Library Example
*/

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "SGP30.h"

static const char *TAG = "sgp30-test";

static void main_task(void *arg) {
    ESP_LOGI(TAG, "SGP30 main task initializing...");

    sgp30_t main_sensor;

    sgp30_init(&main_sensor);

    // SGP30 needs to be read every 1s and sends TVOC = 400 14 times when initializing
    for (int i = 0; i < 14; i++) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        sgp30_IAQ_measure(&main_sensor);
        ESP_LOGI(TAG, "SGP30 Calibrating... TVOC: %d,  eCO2: %d",  main_sensor.TVOC, main_sensor.eCO2);
    }

    // Read initial baselines 
    uint16_t eco2_baseline, tvoc_baseline;
    sgp30_get_IAQ_baseline(&main_sensor, &eco2_baseline, &tvoc_baseline);
    ESP_LOGI(TAG, "BASELINES - TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);


    ESP_LOGI(TAG, "SGP30 main task is running...");
    while(1) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        sgp30_IAQ_measure(&main_sensor);

        ESP_LOGI(TAG, "TVOC: %d,  eCO2: %d",  main_sensor.TVOC, main_sensor.eCO2);
    }
}

void app_main(void)
{
    // esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("SGP30-LIB", ESP_LOG_VERBOSE);
    
    // Creation of main task
    xTaskCreate(main_task, "sgp30_main_test_task", 1024 * 2, (void *)0, 10, NULL);
}
