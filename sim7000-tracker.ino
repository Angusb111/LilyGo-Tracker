/*
        SIM7000-tracker
    Based on LilyGO-T-SIM7000G board
*/

#include "Arduino.h"

#include "src/hardware_configuration.h"

#include <TinyGsmClient.h>

#include "src/common.h"
#include "src/communication.h"
#include "src/gnss.h"
#include "src/log.h"
#include "src/ota.h"
#include "src/platform.h"
#include "src/settings.h"
#include "src/ui.h"
#include "src/util.h"

TinyGsm modem = TinyGsm(MODEM_SERIAL);
TinyGsmClient client = TinyGsmClient(modem);

void callback_helper(char* topic, byte* payload, unsigned int len);

// store system state in RTC memory so that it will be remembered through out sleep
RTC_DATA_ATTR uint16_t bootcount = 0;
RTC_DATA_ATTR wifi_details ota_wifi_details;
RTC_DATA_ATTR ota::status ota_status = ota::status::none;
Settings config = Settings();
platform device = platform();
Ui ui = Ui(&device);
Gnss gnss = Gnss(&modem);
Communication communications = Communication(&modem, &client, &config, callback_helper);

uint32_t last_location_timestamp;

static uint32_t location_min_interval = 1000; //ms
static uint32_t status_interval = 10 * 1000;
static uint32_t setting_request_interval = 15 * 60 * 1000;

uint32_t last_status_timestamp = -status_interval;
uint32_t last_setting_request_timestamp = -setting_request_interval; // request settings at every bootup
TaskHandle_t uiTaskHandle = NULL;

void callback_helper(char* topic, byte* payload, unsigned int len)
{
    communications.mqtt_callback(topic, payload, len);
}

void UiTask(void * parameter) {
    ui.Ui_task();
}

void startUiTask()
{
    xTaskCreate(
        UiTask,       // Task function
        "Ui Task",    // Name of the task (for debugging)
        1000,            // Stack size (in words, not bytes)
        NULL,            // Task input parameter
        2,               // Priority of the task
        &uiTaskHandle     // Task handle
    );
}

void restartUiTask()
{
    if (uiTaskHandle != NULL) {
        vTaskDelete(uiTaskHandle);
        uiTaskHandle = NULL;
    }
    startUiTask();
}

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    INFO("SIM7000-tracker, Eero Silfverberg, 2025");

    // OTA mode
    if (strlen(ota_wifi_details.wifi_ssid) > 0) {
        INFO("Staring OTA");
        ota ota_updater = ota();
        if (ota_updater.try_to_connect_to_wifi(&ota_wifi_details)) {
            INFO("Connected to OTA wifi");
            delay(100);
            ota_status = ota_updater.start();
        } else {
            ota_status = ota::status::wifi_failed;
            ERROR("Failed to connect to wifi");
        }
        strcpy(ota_wifi_details.wifi_ssid, "");
        strcpy(ota_wifi_details.wifi_passwd, "");
        device.restart();
    }
    startUiTask();

    ui.set_state(Ui::state::single_blink);

    if (bootcount != 0) {
        INFO("Woken up from deep sleep");
    }
    device.update();
    communications.init(ota_status);
    ota_status = ota::status::none;
    bootcount++;
}

void loop()
{

    // Always-on tracking
    if (!communications.connected_to_mqtt_broker()) {
        communications.set_state(Communication::modem_state::mqtt_connected);
    }

    if (!gnss.is_on() && !communications.modem_is_off()) {
        gnss.turn_on();
    }

    if (communications.connected_to_mqtt_broker() && gnss.has_fix()) {
        if (util::get_time_diff(last_location_timestamp) > location_min_interval) {
            location_update loc;
            gnss.get_location(&loc);
            communications.send_location(&loc);
            INFO("POSITION SENT!");
            last_location_timestamp = millis();
        }
    } else if (!gnss.has_fix()) {
        INFO("Waiting for GNSS fix");
    }

    // UI states
    if(device.charging() && device.get_soc() == 100)
    {
        ui.set_state(Ui::state::full_on);
    }
    else if(device.charging())
    {
        ui.set_state(Ui::state::half_blink);
    }
    else if(gnss.has_fix())
    {
        ui.set_state(Ui::state::two_blinks);
    }
    else
    {
        ui.set_state(Ui::state::single_blink);
    }

    if (communications.connected_to_mqtt_broker()) {
        if (util::get_time_diff(last_status_timestamp) > status_interval) {
            communications.send_status(
                device.get_soc(),
                device.charging(),
                device.get_voltage()
            );
            last_status_timestamp = millis();
        }
        if (util::get_time_diff(last_setting_request_timestamp) > setting_request_interval) {
            communications.request_settings();
            last_setting_request_timestamp = millis();
        }
    }

    // Updates
    communications.update();
    device.update();
    gnss.update();
}
