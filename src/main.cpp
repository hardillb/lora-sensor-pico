#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "pico/lorawan.h"

#include "bme280.hpp"
#include "common/pimoroni_i2c.hpp"

#include "tusb.h"

#include "config.h"

using namespace pimoroni;

const struct lorawan_sx1276_settings sx1276_settings = {
    .spi = {
        .inst = PICO_DEFAULT_SPI_INSTANCE,
        .mosi = PICO_DEFAULT_SPI_TX_PIN,
        .miso = PICO_DEFAULT_SPI_RX_PIN,
        .sck  = PICO_DEFAULT_SPI_SCK_PIN,
        .nss  = 8
    },
    .reset = 9,
    .dio0  = 7,
    .dio1  = 10
};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui   = LORAWAN_DEVICE_EUI,
    .app_eui      = LORAWAN_APP_EUI,
    .app_key      = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

I2C i2c(BOARD::BREAKOUT_GARDEN);
BME280 bme280(&i2c);


int main() {
    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    if(!bme280.init()) {
        printf("Failed to init bme280!\n");
    }

    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_otaa(&sx1276_settings, LORAWAN_REGION, &otaa_settings) < 0) {
        printf("failed!!!\n");
        while (1) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    // Start the join process and wait
    printf("Joining LoRaWAN network ...");
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process_timeout_ms(1000);
        printf(".");
    }
    printf(" joined successfully!\n");


    while (1) {
        BME280::bme280_reading result = bme280.read_forced();
        printf("%s %0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", result.status == BME280_OK ? "OK" : "ER", result.temperature, result.pressure, result.humidity);


        // send the temperature and humitity byte in an unconfirmed uplink message
        // if (lorawan_send_unconfirmed(&adc_temperature_byte, sizeof(adc_temperature_byte), 2) < 0) {
        //     printf("failed!!!\n");
        // } else {
        //     printf("success!\n");
        // }

         // wait for up to 30 seconds for an event
        if (lorawan_process_timeout_ms(30000) == 0) {
            // check if a downlink message was received
            receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
            if (receive_length > -1) {
                printf("received a %d byte message on port %d: ", receive_length, receive_port);

                for (int i = 0; i < receive_length; i++) {
                    printf("%02x", receive_buffer[i]);
                }
                printf("\n");
            }
        }
    }

  return 0;

}