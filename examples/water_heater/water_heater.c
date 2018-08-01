/*
 * Water_heater example
 * 
 * Water heater with electrical heating element, with a pump to circulate water thru solar panels
 * with support to a diverting valve to passthru water to comsumption or heat with an external heater (example: electrical heater).
 * 
 * It is shown as a thermostat on homekit
 *
 * Wiring is as follows (requires ESP-12 as it needs 4 GPIOs):
 *
 * DHT11 (temperature sensor)
 *
 *               -------------
 *              |GND       VCC|    (These go to control pins of relay)
 *              |15         13| --> Heater
 *              |2          12| --> Cooler
 *              |0          14| --> Fan
 *              |5          16|
 * DHT Data <-- |4       CH_PD|
 *              |RXD       ADC|
 *              |TXD      REST|
 *               -------------
 *              |   |-| |-| | |
 *              | __| |_| |_| |
 *               -------------
 *
 */
#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_system.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <etstimer.h>
#include <esplibs/libmain.h>
#include <FreeRTOS.h>
#include <task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "wifi.h"

//#include <dht/dht.h>
#include <ds18b20/ds18b20.h>


#define LED_PIN 2
#define TEMPERATURE_SENSOR_BUS_PIN 4
#define HEATER_PIN 13
#define PUMP_PIN 99
#define VALVE_PIN 99

#define MAX_SENSORS 3
#define VALVE_THRESHOLD 40
#define DT_PUMP_START 8
#define DT_PUMP_STOP 1
#define TEMP_START_DEFROST 1
#define TEMP_STOP_DEFROST 5
#define TEMP_POLL_PERIOD 10000
//#define HEATER_FAN_DELAY 30000
//#define COOLER_FAN_DELAY 0


static void wifi_init() {
    struct sdk_station_config wifi_config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
    };

    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&wifi_config);
    sdk_wifi_station_connect();
}


void thermostat_identify(homekit_value_t _value) {
    printf("Thermostat identify\n");
}




//ETSTimer fan_timer;


void heaterOn() {
    gpio_write(HEATER_PIN, false);
}


void heaterOff() {
    gpio_write(HEATER_PIN, true);
}

/*
void coolerOn() {
    gpio_write(COOLER_PIN, false);
}


void coolerOff() {
    gpio_write(COOLER_PIN, true);
}


void fan_alarm(void *arg) {
    gpio_write(FAN_PIN, false);
}

void fanOn(uint16_t delay) {
    if (delay > 0) {
        sdk_os_timer_arm(&fan_timer, delay, false);
    } else {
        gpio_write(FAN_PIN, false);
    }
}


void fanOff() {
    sdk_os_timer_disarm(&fan_timer);
    gpio_write(FAN_PIN, true);
}
*/

//void update_state();


void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_state();
}


homekit_characteristic_t current_temperature = HOMEKIT_CHARACTERISTIC_(
    CURRENT_TEMPERATURE, 0
);
homekit_characteristic_t target_temperature  = HOMEKIT_CHARACTERISTIC_(
    TARGET_TEMPERATURE, 45, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
);
homekit_characteristic_t units = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0);
homekit_characteristic_t current_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0);
homekit_characteristic_t target_state = HOMEKIT_CHARACTERISTIC_(
    TARGET_HEATING_COOLING_STATE, 0, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
);
//homekit_characteristic_t cooling_threshold = HOMEKIT_CHARACTERISTIC_(
//    COOLING_THRESHOLD_TEMPERATURE, 25, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
//);
homekit_characteristic_t heating_threshold = HOMEKIT_CHARACTERISTIC_(
    HEATING_THRESHOLD_TEMPERATURE, 40, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
);
//homekit_characteristic_t current_humidity = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);


void update_state() {
    uint8_t state = target_state.value.int_value;
    if ((state == 1 && current_temperature.value.float_value < target_temperature.value.float_value) ||
            (state == 3 && current_temperature.value.float_value < heating_threshold.value.float_value)) {
        if (current_state.value.int_value != 1) {
            current_state.value = HOMEKIT_UINT8(1);
            homekit_characteristic_notify(&current_state, current_state.value);

            heaterOn();
            //coolerOff();
            //fanOff();
            //fanOn(HEATER_FAN_DELAY);
        }
    } /*else if ((state == 2 && current_temperature.value.float_value > target_temperature.value.float_value) ||
            (state == 3 && current_temperature.value.float_value > cooling_threshold.value.float_value)) {
        if (current_state.value.int_value != 2) {
            current_state.value = HOMEKIT_UINT8(2);
            homekit_characteristic_notify(&current_state, current_state.value);

            coolerOn();
            heaterOff();
            fanOff();
            fanOn(COOLER_FAN_DELAY);
        }*/
    } else {
        if (current_state.value.int_value != 0) {
            current_state.value = HOMEKIT_UINT8(0);
            homekit_characteristic_notify(&current_state, current_state.value);

            //coolerOff();
            heaterOff();
            //fanOff();
        }
    }
}


void temperature_sensor_task(void *_args) {
    //sdk_os_timer_setfn(&fan_timer, fan_alarm, NULL);

    ds18b20_addr_t addrs[MAX_SENSORS];
    float temps[MAX_SENSORS];
    int sensor_count;

    gpio_set_pullup(TEMPERATURE_SENSOR_PIN, false, false);

    gpio_enable(HEATER_PIN, GPIO_OUTPUT);
    gpio_enable(PUMP_PIN, GPIO_OUTPUT);
    gpio_enable(VALVE_PIN, GPIO_OUTPUT);

    heaterOff();
    //coolerOff();

    float humidity_value, temperature_value;
    /*
    while (1) {
        bool success = dht_read_float_data(
            DHT_TYPE_DHT11, TEMPERATURE_SENSOR_PIN,
            &humidity_value, &temperature_value
        );
        if (success) {
            printf("Got readings: temperature %g, humidity %g\n", temperature_value, humidity_value);
            current_temperature.value = HOMEKIT_FLOAT(temperature_value);
            current_humidity.value = HOMEKIT_FLOAT(humidity_value);

            homekit_characteristic_notify(&current_temperature, current_temperature.value);
            homekit_characteristic_notify(&current_humidity, current_humidity.value);

            update_state();
        } else {
            printf("Couldnt read data from sensor\n");
        }

        vTaskDelay(TEMPERATURE_POLL_PERIOD / portTICK_PERIOD_MS);
    }*/

    while(1) {
        // Every RESCAN_INTERVAL samples, check to see if the sensors connected
        // to our bus have changed.
        sensor_count = ds18b20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS);

        if (sensor_count < 1) {
            printf("\nNo sensors detected!\n");
        } else {
            printf("\n%d sensors detected:\n", sensor_count);
            // If there were more sensors found than we have space to handle,
            // just report the first MAX_SENSORS..
            if (sensor_count > MAX_SENSORS) sensor_count = MAX_SENSORS;

            // Do a number of temperature samples, and print the results.
            for (int i = 0; i < RESCAN_INTERVAL; i++) {
                ds18b20_measure_and_read_multi(SENSOR_GPIO, addrs, sensor_count, temps);
                for (int j = 0; j < sensor_count; j++) {
                    // The DS18B20 address is a 64-bit integer, but newlib-nano
                    // printf does not support printing 64-bit values, so we
                    // split it up into two 32-bit integers and print them
                    // back-to-back to make it look like one big hex number.
                    uint32_t addr0 = addrs[j] >> 32;
                    uint32_t addr1 = addrs[j];
                    float temp_c = temps[j];
                    float temp_f = (temp_c * 1.8) + 32;
                    printf("  Sensor %08x%08x reports %f deg C (%f deg F)\n", addr0, addr1, temp_c, temp_f);
                }
                printf("\n");

                // Wait for a little bit between each sample (note that the
                // ds18b20_measure_and_read_multi operation already takes at
                // least 750ms to run, so this is on top of that delay).
                vTaskDelay(TEMP_POLL_PERIOD / portTICK_PERIOD_MS);
            }
        }
    }
}

void thermostat_init() {
    xTaskCreate(temperature_sensor_task, "Thermostat", 256, NULL, 2, NULL);
}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Water Heater"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "001"),
            HOMEKIT_CHARACTERISTIC(MODEL, "MyThermostat"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, thermostat_identify),
            NULL
        }),
        HOMEKIT_SERVICE(THERMOSTAT, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Thermostat"),
            &current_temperature,
            &target_temperature,
            &current_state,
            &target_state,
            &heating_threshold,
            &units,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-37-111"
};

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "Dual Lamp-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Dual Lamp-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
}

void user_init(void) {
    uart_set_baud(0, 115200);

    create_accessory_name();

    wifi_init();
    thermostat_init();
    homekit_server_init(&config);
}

