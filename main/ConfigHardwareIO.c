
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ConfigHardwareIO.h"
#include "esp_bit_defs.h"

static const char *TAG = "HardwareIO";
static QueueHandle_t gpio_evt_queue = NULL;
static callback_handler_t callback_handler;
static volatile bool interrupt_happened = false;

//*********************************************
static void IRAM_ATTR gpio_interrupt_callback(void* arg)
{
    if(!interrupt_happened){
        interrupt_happened = true;
        uint32_t gpio_num = (uint32_t) arg;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }
}
//*********************************************
static void gpio_task_thread(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			//ESP_LOGW(TAG, "BtnConfig - gpio_task_thread ");
            printf("gpio_task_thread -> GPIO[%d] intr src\n", io_num);
            if (NULL != callback_handler){
                callback_handler(io_num);
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
            interrupt_happened = false;
        }
    }
}
//*********************************************
uint64_t GetPinBitMask(uint32_t* pPortNums, int count){
    uint64_t GpioSelPins = 0;
    for(int loc = 0; loc < count; loc++){
        GpioSelPins |= BIT64(pPortNums[loc]);
    }
    ESP_LOGI(TAG, "GetPinBitMask 0x%llx", GpioSelPins);
    return GpioSelPins;
}
//*********************************************
void StartQueue(){
    if(NULL ==  gpio_evt_queue){
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
        //start gpio task
        xTaskCreate(gpio_task_thread, "Btn_Handdler_gpio_task", 2048, NULL, 10, NULL);
        ESP_LOGW(TAG, "Queue and xTaskCreate started");
    }
}

//*********************************************
void HwConfigInput(uint32_t* pPortNums, int count){
    uint64_t pin_bit_mask = GetPinBitMask(pPortNums, count);
    gpio_config_t gpioConfig = {
        .pin_bit_mask = pin_bit_mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&gpioConfig);
	ESP_LOGW(TAG, "ConfigInput -> %d", ret);
}
//*********************************************
void HwConfigOutput(uint32_t* pPortNums, int count){
    uint64_t pin_bit_mask = GetPinBitMask(pPortNums, count);
    gpio_config_t gpioConfig = {
        .pin_bit_mask = pin_bit_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&gpioConfig);
	ESP_LOGW(TAG, "ConfigOutput -> %d", ret);
}
//*********************************************
void HwConfigInterrupts(uint32_t* pPortNums, int count, callback_handler_t interrupt_callbackhandler){
    callback_handler = interrupt_callbackhandler;
    StartQueue();
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    uint64_t pin_bit_mask = GetPinBitMask(pPortNums, count);
    gpio_config_t gpioConfig = {
        .pin_bit_mask = pin_bit_mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE
    };
    esp_err_t ret = gpio_config(&gpioConfig);
	ESP_LOGW(TAG, "HwConfigInterrupts -> %d", ret);

    for(int loc = 0; loc < count; loc++){
        uint32_t gpio = pPortNums[loc];
        ret = gpio_isr_handler_add(gpio, gpio_interrupt_callback, (void*)gpio);
        ESP_LOGI(TAG, "HwConfigInterrupts - gpio_isr_handler_add port %d - ret %d", gpio, ret);
    }
    
}


