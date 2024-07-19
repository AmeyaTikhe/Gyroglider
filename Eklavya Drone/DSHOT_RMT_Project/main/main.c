#include <DShotRMT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>

#define GPIO_NUM   GPIO_NUM_18  // Specify your GPIO number
#define RMT_CH     RMT_CHANNEL_0  // Specify your RMT channel

typedef struct {
    gpio_num_t gpio_num;
    uint8_t pin_num;
    rmt_channel_t rmt_channel;
    uint8_t mem_block_num;
    dshot_mode_t mode;
    uint8_t clk_div;
    char *name_str;
    bool is_bidirectional;
    uint8_t ticks_per_bit;
    uint8_t ticks_zero_high;
    uint8_t ticks_one_high;
    uint8_t ticks_zero_low;
    uint8_t ticks_one_low;
} dshot_config_t;

typedef struct {
    uint16_t throttle_value;
    uint8_t telemetric_request;
    uint16_t checksum;
} dshot_packet_t;

typedef struct {
    rmt_item32_t *tx_rmt_item;
    rmt_config_t tx_rmt_config;
    dshot_config_t dshot_config;
} DShotRMT_t;

static void rampThrottle(DShotRMT_t *dshot_rmt, int start, int stop, int step) {
    if (step == 0)
        return;

    for (int i = start; step > 0 ? i < stop : i > stop; i += step) {
        DShotRMT_sendThrottleValue(dshot_rmt, i);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    DShotRMT_sendThrottleValue(dshot_rmt, stop);
}

DShotRMT_t *DShotRMT_new(gpio_num_t gpio, rmt_channel_t rmtChannel) {
    DShotRMT_t *dshot_rmt = malloc(sizeof(DShotRMT_t));
    dshot_rmt->dshot_config.gpio_num = gpio;
    dshot_rmt->dshot_config.pin_num = (uint8_t)gpio;
    dshot_rmt->dshot_config.rmt_channel = rmtChannel;
    dshot_rmt->dshot_config.mem_block_num = RMT_CHANNEL_MAX - (uint8_t)rmtChannel;
    buildTxRmtItem(dshot_rmt, 0); // Initialize with a NULL packet
    return dshot_rmt;
}

DShotRMT_t *DShotRMT_new_pin(uint8_t pin) {
    DShotRMT_t *dshot_rmt = malloc(sizeof(DShotRMT_t));
    dshot_rmt->dshot_config.gpio_num = (gpio_num_t)pin;
    dshot_rmt->dshot_config.pin_num = pin;
    dshot_rmt->dshot_config.rmt_channel = RMT_CHANNEL_MAX - 1;
    dshot_rmt->dshot_config.mem_block_num = RMT_CHANNEL_MAX - 1;
    buildTxRmtItem(dshot_rmt, 0); // Initialize with a NULL packet
    return dshot_rmt;
}

void DShotRMT_delete(DShotRMT_t *dshot_rmt) {
    rmt_driver_uninstall(dshot_rmt->dshot_config.rmt_channel);
    free(dshot_rmt);
}

bool DShotRMT_begin(DShotRMT_t *dshot_rmt, dshot_mode_t dshot_mode, bool is_bidirectional) {
    dshot_rmt->dshot_config.mode = dshot_mode;
    dshot_rmt->dshot_config.clk_div = DSHOT_CLK_DIVIDER;
    dshot_rmt->dshot_config.is_bidirectional = is_bidirectional;

    switch (dshot_mode) {
        case DSHOT150:
            dshot_rmt->dshot_config.ticks_per_bit = 64;
            dshot_rmt->dshot_config.ticks_zero_high = 24;
            dshot_rmt->dshot_config.ticks_one_high = 48;
            break;
        case DSHOT300:
            dshot_rmt->dshot_config.ticks_per_bit = 32;
            dshot_rmt->dshot_config.ticks_zero_high = 12;
            dshot_rmt->dshot_config.ticks_one_high = 24;
            break;
        case DSHOT600:
            dshot_rmt->dshot_config.ticks_per_bit = 16;
            dshot_rmt->dshot_config.ticks_zero_high = 6;
            dshot_rmt->dshot_config.ticks_one_high = 12;
            break;
        case DSHOT1200:
            dshot_rmt->dshot_config.ticks_per_bit = 8;
            dshot_rmt->dshot_config.ticks_zero_high = 3;
            dshot_rmt->dshot_config.ticks_one_high = 6;
            break;
        default:
            return false;
    }

    dshot_rmt->dshot_config.ticks_zero_low = dshot_rmt->dshot_config.ticks_per_bit - dshot_rmt->dshot_config.ticks_zero_high;
    dshot_rmt->dshot_config.ticks_one_low = dshot_rmt->dshot_config.ticks_per_bit - dshot_rmt->dshot_config.ticks_one_high;

    dshot_rmt->tx_rmt_config.rmt_mode = RMT_MODE_TX;
    dshot_rmt->tx_rmt_config.channel = dshot_rmt->dshot_config.rmt_channel;
    dshot_rmt->tx_rmt_config.gpio_num = dshot_rmt->dshot_config.gpio_num;
    dshot_rmt->tx_rmt_config.mem_block_num = dshot_rmt->dshot_config.mem_block_num;
    dshot_rmt->tx_rmt_config.clk_div = dshot_rmt->dshot_config.clk_div;
    dshot_rmt->tx_rmt_config.tx_config.loop_en = false;
    dshot_rmt->tx_rmt_config.tx_config.carrier_en = false;
    dshot_rmt->tx_rmt_config.tx_config.idle_output_en = true;

    if (dshot_rmt->dshot_config.is_bidirectional) {
        dshot_rmt->tx_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    } else {
        dshot_rmt->tx_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    }

    rmt_config(&dshot_rmt->tx_rmt_config);

    return rmt_driver_install(dshot_rmt->tx_rmt_config.channel, 0, 0) == ESP_OK;
}

void DShotRMT_sendThrottleValue(DShotRMT_t *dshot_rmt, uint16_t throttle_value) {
    dshot_packet_t dshot_rmt_packet = {};

    if (throttle_value < DSHOT_THROTTLE_MIN) {
        throttle_value = DSHOT_THROTTLE_MIN;
    }

    if (throttle_value > DSHOT_THROTTLE_MAX) {
        throttle_value = DSHOT_THROTTLE_MAX;
    }

    dshot_rmt_packet.throttle_value = throttle_value;
    dshot_rmt_packet.telemetric_request = 0;  // NO_TELEMETRIC

    dshot_rmt_packet.checksum = calculateCRC(dshot_rmt_packet);

    sendRmtPaket(dshot_rmt, dshot_rmt_packet);
}

rmt_item32_t *buildTxRmtItem(DShotRMT_t *dshot_rmt, uint16_t parsed_packet) {
    dshot_rmt->tx_rmt_item = malloc((DSHOT_PAUSE_BIT + 1) * sizeof(rmt_item32_t)); // +1 for pause bit
    if (dshot_rmt->dshot_config.is_bidirectional) {
        for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) {
            if (parsed_packet & 0b1000000000000000) {
                dshot_rmt->tx_rmt_item[i].duration0 = dshot_rmt->dshot_config.ticks_one_low;
                dshot_rmt->tx_rmt_item[i].duration1 = dshot_rmt->dshot_config.ticks_one_high;
            } else {
                dshot_rmt->tx_rmt_item[i].duration0 = dshot_rmt->dshot_config.ticks_zero_low;
                dshot_rmt->tx_rmt_item[i].duration1 = dshot_rmt->dshot_config.ticks_zero_high;
            }
            dshot_rmt->tx_rmt_item[i].level0 = 0;
            dshot_rmt->tx_rmt_item[i].level1 = 1;
        }
    } else {
        for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) {
            if (parsed_packet & 0b1000000000000000) {
                dshot_rmt->tx_rmt_item[i].duration0 = dshot_rmt->dshot_config.ticks_one_high;
                dshot_rmt->tx_rmt_item[i].duration1 = dshot_rmt->dshot_config.ticks_one_low;
            } else {
                dshot_rmt->tx_rmt_item[i].duration0 = dshot_rmt->dshot_config.ticks_zero_high;
                dshot_rmt->tx_rmt_item[i].duration1 = dshot_rmt->dshot_config.ticks_zero_low;
            }
            dshot_rmt->tx_rmt_item[i].level0 = 1;
            dshot_rmt->tx_rmt_item[i].level1 = 0;
        }
    }

    // DSHOT_PAUSE_BIT pause bit
    dshot_rmt->tx_rmt_item[DSHOT_PAUSE_BIT].duration0 = 4 * dshot_rmt->dshot_config.ticks_per_bit;
    dshot_rmt->tx_rmt_item[DSHOT_PAUSE_BIT].duration1 = 0;
    dshot_rmt->tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 0;
    dshot_rmt->tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 1;

    return dshot_rmt->tx_rmt_item;
}

uint16_t calculateCRC(dshot_packet_t packet) {
    uint16_t crc = packet.throttle_value << 1 | packet.telemetric_request;
    uint16_t csum_data = crc;
    uint16_t csum = 0;

    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    return (crc << 4) | (csum & 0xf);
}

void sendRmtPaket(DShotRMT_t *dshot_rmt, dshot_packet_t packet) {
    uint16_t dshot_packet = ((packet.throttle_value << 1) | packet.telemetric_request) << 4 | packet.checksum;
    rmt_write_items(dshot_rmt->dshot_config.rmt_channel, buildTxRmtItem(dshot_rmt, dshot_packet), DSHOT_PAUSE_BIT + 1, true);
}

void app_main(void) {
    DShotRMT_t *dshot_rmt = DShotRMT_new(GPIO_NUM, RMT_CH);

    if (DShotRMT_begin(dshot_rmt, DSHOT600, false)) {
        printf("DShot ESC initialized successfully\n");
    } else {
        printf("Failed to initialize DShot ESC\n");
        return;
    }

    // Ramp throttle from min to max and then back to min
    rampThrottle(dshot_rmt, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX, 20);
    rampThrottle(dshot_rmt, DSHOT_THROTTLE_MAX, DSHOT_THROTTLE_MIN, -20);

    DShotRMT_delete(dshot_rmt);
}
