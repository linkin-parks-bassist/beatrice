#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdio.h>

#include "beat_detection.h"
#include "led_control.h"
#include "buffers.h"
#include "mic.h"

// Run  all initialisation routines and report any errors
int init()
{
	int error;
	
	if ((error = init_mic()))
		return error;
	
	if ((error = init_leds()))
		return error;
	
	if ((error = init_buffer_queue()))
		return error;
	
	return 0;
}

#define WAKEUP_BUTTON GPIO_NUM_3   // pin 3 -> button -> GND

void button_task(void *arg)
{
    for (;;) {
        if (gpio_get_level(WAKEUP_BUTTON) == 0) { // button pressed
            vTaskDelay(pdMS_TO_TICKS(50));        // debounce
            if (gpio_get_level(WAKEUP_BUTTON) == 0) {
                printf("Going to deep sleep now...\n");

                // Arm wakeup *before* sleeping
                esp_sleep_enable_ext0_wakeup(WAKEUP_BUTTON, 0);

                // Optional: wait until button is released, so we don't wake immediately
                while (gpio_get_level(WAKEUP_BUTTON) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }

                esp_deep_sleep_start();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    // --- Check why we woke up ---
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        ESP_LOGI("WAKE", "Woke up from button press on GPIO3");

        // Wait for button release after wakeup to avoid instant re-sleep
        while (gpio_get_level(WAKEUP_BUTTON) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    } else {
        ESP_LOGI("WAKE", "Cold boot (normal power-on)");
    }

    // --- Configure the button pin as input with pull-up ---
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << WAKEUP_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Always arm wakeup (redundant here since we also do it in button_task, but harmless)
    esp_sleep_enable_ext0_wakeup(WAKEUP_BUTTON, 0);

    int error;
    if ((error = init())) {
        // If there is an error, report it and die
        printf("Error %d\n", error);
    } else {
        // If there is no error, report successful initialisation
        printf("Initialised\n");

        xTaskCreatePinnedToCore(beat_detection_task, "beat_detector", 16384, NULL, 5, NULL, 0);
        xTaskCreatePinnedToCore(button_task,        "button_task",   2048,  NULL, 5, NULL, 0);
        xTaskCreatePinnedToCore(mic_read_task,      "mic_reader",    4096,  NULL, 5, NULL, 1);
    }
}
