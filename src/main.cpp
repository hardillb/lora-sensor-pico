#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"

// #include "pico/lorawan.h"
extern "C" {
#include "pico/lorawan.h"
}

#include "bme280.hpp"
#include "common/pimoroni_i2c.hpp"

#include "tusb.h"

#include "config.h"

#define hiByte(i) (i & 0xff00)>>8
#define lowByte(i) (i & 0xff)

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

uint16_t payload[4];

I2C i2c(BOARD::BREAKOUT_GARDEN);
BME280 bme280(&i2c);

uint16_t f2sflt16(float f);

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

        float temp_f = result.temperature/100;
        float humidity_f = result.humidity/100;

        uint16_t temp_i = f2sflt16(temp_f);
        uint16_t humidity_i = f2sflt16(humidity_f);

        payload[0] = lowByte(temp_i);
        payload[1] = hiByte(temp_i);

        payload[2] = lowByte(humidity_i);
        payload[3] = hiByte(humidity_i);

        // send the temperature and humitity byte in an unconfirmed uplink message
        if (lorawan_send_unconfirmed(&payload, sizeof(payload), 1) < 0) {
            printf("failed!!!\n");
        } else {
            printf("success!\n");
        }

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

uint16_t f2sflt16(float f)
{
    if (f <= -1.0)
        return 0xFFFF;
    else if (f >= 1.0)
        return 0x7FFF;
    else
    {
        int iExp;
        float normalValue;
        uint16_t sign;

        normalValue = frexpf(f, &iExp);

        sign = 0;
        if (normalValue < 0)
        {
            // set the "sign bit" of the result
            // and work with the absolute value of normalValue.
            sign = 0x8000;
            normalValue = -normalValue;
        }

        // abs(f) is supposed to be in [0..1), so useful exp
        // is [0..-15]
        iExp += 15;
        if (iExp < 0)
            iExp = 0;

        // bit 15 is the sign
        // bits 14..11 are the exponent
        // bits 10..0 are the fraction
        // we conmpute the fraction and then decide if we need to round.
        uint16_t outputFraction = ldexpf(normalValue, 11) + 0.5;
        if (outputFraction >= (1 << 11u))
        {
            // reduce output fraction
            outputFraction = 1 << 10;
            // increase exponent
            ++iExp;
        }

        // check for overflow and return max instead.
        if (iExp > 15)
            return 0x7FFF | sign;

        return (uint16_t)(sign | (iExp << 11u) | outputFraction);
    }
}