// Copyright (c) 2015-2019 The HomeKit ADK Contributors
//
// Licensed under the Apache License, Version 2.0 (the “License”);
// you may not use this file except in compliance with the License.
// See [CONTRIBUTORS.md] for the list of HomeKit ADK project authors.

// An example that implements the light bulb HomeKit profile. It can serve as a basic implementation for
// any platform. The accessory logic implementation is reduced to internal state updates and log output.
//
// This implementation is platform-independent.
//
// The code consists of multiple parts:
//
//   1. The definition of the accessory configuration and its internal state.
//
//   2. Helper functions to load and save the state of the accessory.
//
//   3. The definitions for the HomeKit attribute database.
//
//   4. The callbacks that implement the actual behavior of the accessory, in this
//      case here they merely access the global accessory state variable and write
//      to the log to make the behavior easily observable.
//
//   5. The initialization of the accessory state.
//
//   6. Callbacks that notify the server in case their associated value has changed.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "HAP.h"

#include "App.h"
#include "DB.h"
#include "Led.h"
#include "esp_log.h"



static const char *TAG = "HomeKit";
#define LED_GPIO 2
#define BTH_GPIO GPIO_NUM_12

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Domain used in the key value store for application data.
 *
 * Purged: On factory reset.
 */
#define kAppKeyValueStoreDomain_Configuration ((HAPPlatformKeyValueStoreDomain) 0x00)

/**
 * Key used in the key value store to store the configuration state.
 *
 * Purged: On factory reset.
 */
#define kAppKeyValueStoreKey_Configuration_State ((HAPPlatformKeyValueStoreDomain) 0x00)

typedef struct {
    uint8_t pin;
    uint8_t action;
} ButtonEvent;

/**
 * Global accessory configuration.
 */
// typedef struct {
//     struct {
//         bool lightbulbOn;
//     } state;
//     struct {
//         uint32_t lightbulbPin;
//         uint32_t identifyPin;
//     } device;
//     bool restoreFactorySettingsRequested;
//     HAPAccessoryServerRef *server;
//     HAPPlatformTimerRef identifyTimer;
//     HAPPlatformKeyValueStoreRef keyValueStore;
// } AccessoryConfiguration;

/**
 * Global accessory configuration.
 */
typedef struct {
    struct {
        bool lightBulbOn;
    } state;
    struct {
        uint32_t lightbulbPin;
        uint32_t SwitchPin;
    } device;
    HAPPlatformTimerRef identifyTimer;
    HAPAccessoryServerRef* server;
    HAPPlatformKeyValueStoreRef keyValueStore;
} AccessoryConfiguration;

static AccessoryConfiguration accessoryConfiguration;


static void HandleButtonEventCallback(
    void *_Nullable context,
    size_t contextSize)
{
    HAPPrecondition(context);
    HAPAssert(contextSize == sizeof (ButtonEvent));
    
    HAPLogInfo(&kHAPLog_Default, "%s", __func__);
    
    //ButtonEvent buttonEvent = *((ButtonEvent *) context);
    // if (buttonEvent.action == APP_BUTTON_RELEASE) {
    //     switch (buttonEvent.pin) {
    //         case BUTTON_1: {
    //             ToggleLightbulbState();
    //         } break;
    //         case BUTTON_2: {
    //             RestoreFactorySettings();
    //         } break;
    //         default: {
    //         } break;
    //     }
    // }
}
//----------------------------------------------------------------------------------------------------------------------
static bool SetLightStatus(bool newlightBulbStatus)
{
    if (accessoryConfiguration.state.lightBulbOn != newlightBulbStatus) {
        gpio_set_level(accessoryConfiguration.device.lightbulbPin, newlightBulbStatus ? 1 : 0);
        accessoryConfiguration.state.lightBulbOn = newlightBulbStatus;
        return true;
    }
    return false;
}
static void updateHAPCurrentState(void* _Nullable context, size_t contextSize)
{
    ESP_LOGW("STATE ***", "updateHAPCurrentState called !!!!!!!");
    //HAPAccessoryServerRaiseEvent
    //HAPAccessoryServerRaiseEvent(accessoryConfiguration.server, &lockMechanismLockCurrentStateCharacteristic, &lockMechanismService, AppGetAccessoryInfo());
}
void ConfigureIO(void);

static void HandleButtonEvent(
    uint8_t pin_no,
    uint8_t button_action)
{
    HAPError err;
    ButtonEvent buttonEvent = {
        .pin = pin_no,
        .action = button_action
    };
    
    err = HAPPlatformRunLoopScheduleCallback(HandleButtonEventCallback, &buttonEvent, sizeof buttonEvent);
    if (err) {
        HAPFatalError();
    }
}

//https://github.com/espressif/esp-apple-homekit-adk/issues/4

// static void updateHAPCurrentState(void* _Nullable context, size_t contextSize)
// {
//     ESP_LOGW(TAG, "updateHAPCurrentState called");
//     //HAPAccessoryServerRaiseEvent(accessoryConfiguration.server, &lockMechanismLockCurrentStateCharacteristic, &lockMechanismService, AppGetAccessoryInfo());
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//----------------------------------------------------------------------------------------------------------------------

/**
 * Load the accessory state from persistent memory.
 */
static void LoadAccessoryState(void) {
    HAPPrecondition(accessoryConfiguration.keyValueStore);

    HAPError err;

    // Load persistent state if available
    bool found;
    size_t numBytes;

    err = HAPPlatformKeyValueStoreGet(
            accessoryConfiguration.keyValueStore,
            kAppKeyValueStoreDomain_Configuration,
            kAppKeyValueStoreKey_Configuration_State,
            &accessoryConfiguration.state,
            sizeof accessoryConfiguration.state,
            &numBytes,
            &found);

    if (err) {
        HAPAssert(err == kHAPError_Unknown);
        HAPFatalError();
    }
    if (!found || numBytes != sizeof accessoryConfiguration.state) {
        if (found) {
            HAPLogError(&kHAPLog_Default, "Unexpected app state found in key-value store. Resetting to default.");
        }
        HAPRawBufferZero(&accessoryConfiguration.state, sizeof accessoryConfiguration.state);
    }
}

/**
 * Save the accessory state to persistent memory.
 */
static void SaveAccessoryState(void) {
    HAPPrecondition(accessoryConfiguration.keyValueStore);

    HAPError err;
    err = HAPPlatformKeyValueStoreSet(
            accessoryConfiguration.keyValueStore,
            kAppKeyValueStoreDomain_Configuration,
            kAppKeyValueStoreKey_Configuration_State,
            &accessoryConfiguration.state,
            sizeof accessoryConfiguration.state);
    if (err) {
        HAPAssert(err == kHAPError_Unknown);
        HAPFatalError();
    }
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * HomeKit accessory that provides the Light Bulb service.
 *
 * Note: Not constant to enable BCT Manual Name Change.
 */
static HAPAccessory accessory = { .aid = 1,
                                  .category = kHAPAccessoryCategory_Lighting,
                                  .name = "ESP32 LED",
                                  .manufacturer = "Esp32Dev",
                                  .model = "LightBulb1,1",
                                  .serialNumber = "099DB48E9E30",
                                  .firmwareVersion = "1",
                                  .hardwareVersion = "1",
                                  .services = (const HAPService* const[]) { &accessoryInformationService,
                                                                            &hapProtocolInformationService,
                                                                            &pairingService,
                                                                            &lightBulbService,
                                                                            NULL },
                                  .callbacks = { .identify = IdentifyAccessory } };


//----------------------------------------------------------------------------------------------------------------------

HAP_RESULT_USE_CHECK
HAPError IdentifyAccessory(
        HAPAccessoryServerRef* server HAP_UNUSED,
        const HAPAccessoryIdentifyRequest* request HAP_UNUSED,
        void* _Nullable context HAP_UNUSED) {
    HAPLogInfo(&kHAPLog_Default, "%s", __func__);
    return kHAPError_None;
}

HAP_RESULT_USE_CHECK
HAPError HandleLightBulbOnRead(
        HAPAccessoryServerRef* server HAP_UNUSED,
        const HAPBoolCharacteristicReadRequest* request HAP_UNUSED,
        bool* value,
        void* _Nullable context HAP_UNUSED) {
    *value = accessoryConfiguration.state.lightBulbOn;
    ESP_LOGW(TAG, "HandleLightBulbOnRead");
    HAPLogInfo(&kHAPLog_Default, "%s: %s", __func__, *value ? "true" : "false");

    return kHAPError_None;
}

HAP_RESULT_USE_CHECK
HAPError HandleLightBulbOnWrite(
        HAPAccessoryServerRef* server,
        const HAPBoolCharacteristicWriteRequest* request,
        bool value,
        void* _Nullable context HAP_UNUSED) {
    HAPLogInfo(&kHAPLog_Default, "%s: %s", __func__, value ? "true" : "false");
    ESP_LOGW(TAG, "HandleLightBulbOnWrite");
    if (SetLightStatus(value))
    {
        HAPAccessoryServerRaiseEvent(server, request->characteristic, request->service, request->accessory);
    }

    return kHAPError_None;
}
HAP_RESULT_USE_CHECK
HAPError HandleSwitchOnSubscribe(
        HAPAccessoryServerRef* server,
        const HAPBoolCharacteristicWriteRequest* request,
        bool value,
        void* _Nullable context)
        {
            ESP_LOGW(TAG, "HandleSwitchOnSubscribe");
            return kHAPError_None;
        }
        
HAP_RESULT_USE_CHECK
HAPError HandleSwitchOnUnsubscribe(
        HAPAccessoryServerRef* server,
        const HAPBoolCharacteristicWriteRequest* request,
        bool value,
        void* _Nullable context)
        {
            ESP_LOGW(TAG, "HandleSwitchOnUnsubscribe");
            return kHAPError_None;
        }
//----------------
void ToggleLightStatus()
{
    bool curr = accessoryConfiguration.state.lightBulbOn;
    SetLightStatus(!curr);
}
//----------------------------------------------------------------------------------------------------------------------

void AccessoryNotification(
        const HAPAccessory* accessory,
        const HAPService* service,
        const HAPCharacteristic* characteristic,
        void* ctx) {
    HAPLogInfo(&kHAPLog_Default, "Accessory Notification");

    HAPAccessoryServerRaiseEvent(accessoryConfiguration.server, characteristic, service, accessory);
}

void AppCreate(HAPAccessoryServerRef* server, HAPPlatformKeyValueStoreRef keyValueStore) {
    HAPPrecondition(server);
    HAPPrecondition(keyValueStore);

    HAPLogInfo(&kHAPLog_Default, "%s", __func__);

    HAPRawBufferZero(&accessoryConfiguration, sizeof accessoryConfiguration);
    accessoryConfiguration.server = server;
    accessoryConfiguration.keyValueStore = keyValueStore;

    ConfigureIO();

    LoadAccessoryState();
}

void AppRelease(void) {
}

void AppAccessoryServerStart(void) {
    HAPAccessoryServerStart(accessoryConfiguration.server, &accessory);
       ESP_LOGW(TAG, "AppAccessoryServerStart");
}

//----------------------------------------------------------------------------------------------------------------------

void AccessoryServerHandleUpdatedState(HAPAccessoryServerRef* server, void* _Nullable context) {
    HAPPrecondition(server);
    HAPPrecondition(!context);

    ESP_LOGW(TAG, "AccessoryServerHandleUpdatedState ******");
    switch (HAPAccessoryServerGetState(server)) {
        case kHAPAccessoryServerState_Idle: {
            HAPLogInfo(&kHAPLog_Default, "Accessory Server State did update: Idle.");
           ESP_LOGW(TAG, "HAPAccessoryServerGetState - 1");
             return;
        }
        case kHAPAccessoryServerState_Running: {
            HAPLogInfo(&kHAPLog_Default, "Accessory Server State did update: Running.");
            // ESP_LOGW(TAG, "HAPAccessoryServerGetState - 2");
            // HAPError err = HAPPlatformRunLoopScheduleCallback(updateHAPCurrentState, NULL, 0);
            // if( kHAPError_None != err)
            // {
            //     ESP_LOGW(TAG, "Failed -- HAPPlatformRunLoopScheduleCallback");
            // }
            // else
            // {
            //     ESP_LOGW(TAG, "Passed -- HAPPlatformRunLoopScheduleCallback");
            // }
            return;
        }
        case kHAPAccessoryServerState_Stopping: {
            HAPLogInfo(&kHAPLog_Default, "Accessory Server State did update: Stopping.");
            return;
        }
    }
    HAPFatalError();
}

const HAPAccessory* AppGetAccessoryInfo() {
    return &accessory;
}

void AppInitialize(
        HAPAccessoryServerOptions* hapAccessoryServerOptions,
        HAPPlatform* hapPlatform,
        HAPAccessoryServerCallbacks* hapAccessoryServerCallbacks) {
    /*no-op*/
    ESP_LOGW(TAG, "AppInitialize");
}

void AppDeinitialize() {
    /*no-op*/
}
//https://github.com/espressif/esp-apple-homekit-adk/issues/4
//https://github.com/apple/HomeKitADK/issues/78
//https://github.com/espressif/esp-apple-homekit-adk/issues/4

// static void updateHAPCurrentState(void* _Nullable context, size_t contextSize)
// {
//     ESP_LOGW(TAG, "updateHAPCurrentState called");
//     //HAPAccessoryServerRaiseEvent(accessoryConfiguration.server, &lockMechanismLockCurrentStateCharacteristic, &lockMechanismService, AppGetAccessoryInfo());
// }

// You will have to implement a callback function that can be added to the runloop which satisfies this type:
// typedef void (*HAPPlatformRunLoopCallback)(void* _Nullable context, size_t contextSize);
// Within this function you may raise the HAP event which will in turn query the read functions for the characteristic.
// in my case the function looks like this:


// You can trigger an execution of your function by adding it to the runloop:
//  should look into if all else fails (arduino).. homespan
// HAPError err = HAPPlatformRunLoopScheduleCallback(updateHAPCurrentState, NULL, 0);


/// good one to read 
// https://github.com/marcodeltutto/esp-apple-homekit-weather-station
//  https://github.com/tijunoi/esp-adk-garage-remote

/// really cool - https://github.com/wwxxyx/Quectel_BG96

// void gpioCallback( void *arg )
// {
//     ESP_LOGI(TAG, "gpioCallback");
// }
volatile bool interrupt_happened = false;
static QueueHandle_t gpio_evt_queue = NULL;


static void gpio_task_thread(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            bool lightBulbStatus = gpio_get_level(io_num) == 1 ? true : false;
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (SetLightStatus(lightBulbStatus))
            {
                //trigger update
                //HAPPlatformRunLoopScheduleCallback(updateHAPCurrentState, NULL, 0);
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            interrupt_happened = false;
        }
    }
}

void IRAM_ATTR gpioCallback(void* arg)
{
    if(!interrupt_happened)
    {
        interrupt_happened = true;
        uint32_t gpio_num = (uint32_t) arg;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }
}
/**
 * Configure platform specific IO.
 */
void ConfigureIO(void)
{
    ESP_LOGI(TAG, "ConfigureIO");
    HAPLogInfo(&kHAPLog_Default, "%s", __func__);
    
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_thread, "gpio_task_example", 2048, NULL, 10, NULL);
 
    // Configure GPIO pins.
    accessoryConfiguration.device.lightbulbPin = LED_GPIO;
    accessoryConfiguration.device.SwitchPin = BTH_GPIO;
    
 
    gpio_pad_select_gpio(accessoryConfiguration.device.lightbulbPin);
    gpio_set_direction(accessoryConfiguration.device.lightbulbPin, GPIO_MODE_OUTPUT);
    gpio_set_level(accessoryConfiguration.device.lightbulbPin, 0);    //gpio_set_level(accessoryConfiguration.device.lightbulbPin, 0);
    
    // intr_handle_t ih;
    // esp_err_t err = esp_intr_alloc(gpioCallback,
    //                                ESP_INTR_FLAG_IRAM, &dummy, NULL, &ih);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    esp_err_t ret = ESP_OK;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1<<BTH_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ret = gpio_config(&io_conf);
    int dd = BTH_GPIO;
    gpio_isr_handler_add(BTH_GPIO, gpioCallback, (void*) &dd);

    // gpio_pad_select_gpio(BTH_GPIO);
    // gpio_set_direction(BTH_GPIO, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(BTH_GPIO, GPIO_PULLUP_ONLY);
    // gpio_set_intr_type(BTH_GPIO, GPIO_INTR_NEGEDGE);
    // gpio_intr_enable(BTH_GPIO);
    //gpio_isr_handle_t handle;
    //esp_err_t ret = gpio_isr_register( gpioCallback,BTH_GPIO, GPIO_INTR_NEGEDGE, &handle);
    //ESP_LOGI(TAG, "ConfigureIO -> %d", ret);
    // gpio_pad_select_gpio(accessoryConfiguration.device.SwitchPin);
    // gpio_set_direction(accessoryConfiguration.device.SwitchPin, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(accessoryConfiguration.device.SwitchPin,GPIO_PULLUP_ONLY);
    // gpio_isr_register(accessoryConfiguration.device.SwitchPin, gpioCallback, GPIO_INTR_NEGEDGE,(void *)TAG);
    
    // Configure buttons
    // static app_button_cfg_t buttonConfigs[] = {
    //     {
    //         .pin_no = BUTTON_1,
    //         .active_state = APP_BUTTON_ACTIVE_LOW,
    //         .pull_cfg = NRF_GPIO_PIN_PULLUP,
    //         .button_handler = HandleButtonEvent
    //     },
    //     // {
    //     //     .pin_no = BUTTON_2,
    //     //     .active_state = APP_BUTTON_ACTIVE_LOW,
    //     //     .pull_cfg = NRF_GPIO_PIN_PULLUP,
    //     //     .button_handler = HandleButtonEvent
    //     // }
    // };

    // uint32_t e = app_button_init(buttonConfigs, HAPArrayCount(buttonConfigs), /* detection_delay: */ 1000);
    // if (e) {
    //     HAPLogError(&kHAPLog_Default, "app_button_init failed: 0x%04x.", (unsigned int) e);
    //     HAPFatalError();
    // }
    // e = app_button_enable();
    // if (e) {
    //     HAPLogError(&kHAPLog_Default, "app_button_enable failed: 0x%04x.", (unsigned int) e);
    //     HAPFatalError();
    // }
}

