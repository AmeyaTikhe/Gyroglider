#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"

static const char *TAG = "dshot-rmt";

#define DSHOT_ERROR_CHECK(x) ({        \
    esp_err_t __ret = x;               \
    if (__ret != ESP_OK)               \
        return __ret;                  \
    __ret;                             \
})

// from https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt
typedef enum {
    DIGITAL_CMD_MOTOR_STOP,
    DIGITAL_CMD_BEEP1,
    DIGITAL_CMD_BEEP2,
    DIGITAL_CMD_BEEP3,
    DIGITAL_CMD_BEEP4,
    DIGITAL_CMD_BEEP5,
    DIGITAL_CMD_ESC_INFO,
    DIGITAL_CMD_SPIN_DIRECTION_1,
    DIGITAL_CMD_SPIN_DIRECTION_2,
    DIGITAL_CMD_3D_MODE_OFF,
    DIGITAL_CMD_3D_MODE_ON,
    DIGITAL_CMD_SETTINGS_REQUEST,
    DIGITAL_CMD_SAVE_SETTINGS,
    DIGITAL_CMD_SPIN_DIRECTION_NORMAL = 20,
    DIGITAL_CMD_SPIN_DIRECTION_REVERSED,
    DIGITAL_CMD_LED0_ON,
    DIGITAL_CMD_LED1_ON,
    DIGITAL_CMD_LED2_ON,
    DIGITAL_CMD_LED3_ON,
    DIGITAL_CMD_LED0_OFF,
    DIGITAL_CMD_LED1_OFF,
    DIGITAL_CMD_LED2_OFF,
    DIGITAL_CMD_LED3_OFF,
} dshot_cmd_t;

// DSHOT Timings
#define DSHOT_TICKS_PER_BIT 19

#define DSHOT_T0H 7
#define DSHOT_T0L (DSHOT_TICKS_PER_BIT - DSHOT_T0H)

#define DSHOT_T1H 14
#define DSHOT_T1L (DSHOT_TICKS_PER_BIT - DSHOT_T1H)

#define DSHOT_PAUSE (DSHOT_TICKS_PER_BIT * 200)
// !DSHOT Timings

#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047

#define DSHOT_ARM_DELAY (5000 / portTICK_PERIOD_MS)

typedef struct {
    rmt_channel_t rmtChannel;
    rmt_item32_t _dshotCmd[17];
} DShotRMT_t;

void DShotRMT_init(DShotRMT_t *dshot);
void DShotRMT_deinit(DShotRMT_t *dshot);
esp_err_t DShotRMT_install(DShotRMT_t *dshot, gpio_num_t gpio, rmt_channel_t rmtChannel);
esp_err_t DShotRMT_uninstall(DShotRMT_t *dshot);
esp_err_t DShotRMT_init_esc(DShotRMT_t *dshot, bool wait);
esp_err_t DShotRMT_sendThrottle(DShotRMT_t *dshot, uint16_t throttle);
esp_err_t DShotRMT_setReversed(DShotRMT_t *dshot, bool reversed);
esp_err_t DShotRMT_beep(DShotRMT_t *dshot);
void DShotRMT_setData(DShotRMT_t *dshot, uint16_t data);
uint8_t DShotRMT_checksum(uint16_t data);
esp_err_t DShotRMT_writeData(DShotRMT_t *dshot, uint16_t data, bool wait);
esp_err_t DShotRMT_writePacket(DShotRMT_t *dshot, uint16_t payload, bool telemetry, bool wait);
esp_err_t DShotRMT_repeatPacket(DShotRMT_t *dshot, uint16_t payload, bool telemetry, int n);
esp_err_t DShotRMT_repeatPacketTicks(DShotRMT_t *dshot, uint16_t payload, bool telemetry, TickType_t ticks);

void DShotRMT_init(DShotRMT_t *dshot) {
    // initialize cmd buffer
    DShotRMT_setData(dshot, 0);

    // DShot packet delay + RMT end marker
    dshot->_dshotCmd[16].duration0 = DSHOT_PAUSE;
    dshot->_dshotCmd[16].level0 = 0;
    dshot->_dshotCmd[16].duration1 = 0;
    dshot->_dshotCmd[16].level1 = 0;
}

void DShotRMT_deinit(DShotRMT_t *dshot) {
    // TODO write destructor equivalent in C if needed
}

esp_err_t DShotRMT_install(DShotRMT_t *dshot, gpio_num_t gpio, rmt_channel_t rmtChannel) {
    dshot->rmtChannel = rmtChannel;

    rmt_config_t config;
    config.channel = rmtChannel;
    config.rmt_mode = RMT_MODE_TX;
    config.gpio_num = gpio;
    config.mem_block_num = 1;
    config.clk_div = 7;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;

    DSHOT_ERROR_CHECK(rmt_config(&config));

    return rmt_driver_install(rmtChannel, 0, 0);
}

esp_err_t DShotRMT_uninstall(DShotRMT_t *dshot) {
    // TODO implement uninstall
    return ESP_OK;
}

esp_err_t DShotRMT_init_esc(DShotRMT_t *dshot, bool wait) {
    ESP_LOGD(TAG, "Sending reset command");
    for (int i = 0; i < 50; i++) {
        DShotRMT_writeData(dshot, 0, true);
    }

    ESP_LOGD(TAG, "Sending idle throttle");
    if (wait) {
        DSHOT_ERROR_CHECK(DShotRMT_repeatPacketTicks(dshot, DSHOT_THROTTLE_MIN, 0, DSHOT_ARM_DELAY));
    } else {
        DShotRMT_writePacket(dshot, DSHOT_THROTTLE_MIN, 0, false);
    }

    ESP_LOGD(TAG, "ESC armed");
    return ESP_OK;
}

esp_err_t DShotRMT_sendThrottle(DShotRMT_t *dshot, uint16_t throttle) {
    if (throttle < DSHOT_THROTTLE_MIN || throttle > DSHOT_THROTTLE_MAX)
        return ESP_ERR_INVALID_ARG;

    return DShotRMT_writePacket(dshot, throttle, 0, false);
}

esp_err_t DShotRMT_setReversed(DShotRMT_t *dshot, bool reversed) {
    DSHOT_ERROR_CHECK(rmt_wait_tx_done(dshot->rmtChannel, 1));
    DSHOT_ERROR_CHECK(DShotRMT_repeatPacketTicks(dshot, DSHOT_THROTTLE_MIN, 0, 200 / portTICK_PERIOD_MS));
    DSHOT_ERROR_CHECK(DShotRMT_repeatPacket(
        dshot,
        reversed ? DIGITAL_CMD_SPIN_DIRECTION_REVERSED : DIGITAL_CMD_SPIN_DIRECTION_NORMAL,
        1, 10
    ));
    return ESP_OK;
}

esp_err_t DShotRMT_beep(DShotRMT_t *dshot) {
    DSHOT_ERROR_CHECK(DShotRMT_writePacket(dshot, DIGITAL_CMD_BEEP1, 1, true));
    vTaskDelay(260 / portTICK_PERIOD_MS);
    return ESP_OK;
}

void DShotRMT_setData(DShotRMT_t *dshot, uint16_t data) {
    for (int i = 0; i < 16; i++, data <<= 1) {
        if (data & 0x8000) {
            // set one
            dshot->_dshotCmd[i].duration0 = DSHOT_T1H;
            dshot->_dshotCmd[i].level0 = 1;
            dshot->_dshotCmd[i].duration1 = DSHOT_T1L;
            dshot->_dshotCmd[i].level1 = 0;
        } else {
            // set zero
            dshot->_dshotCmd[i].duration0 = DSHOT_T0H;
            dshot->_dshotCmd[i].level0 = 1;
            dshot->_dshotCmd[i].duration1 = DSHOT_T0L;
            dshot->_dshotCmd[i].level1 = 0;
        }
    }
}

uint8_t DShotRMT_checksum(uint16_t data) {
    uint16_t csum = 0;

    for (int i = 0; i < 3; i++) {
        csum ^= data;
        data >>= 4;
    }

    return csum & 0xf;
}

esp_err_t DShotRMT_writeData(DShotRMT_t *dshot, uint16_t data, bool wait) {
    DSHOT_ERROR_CHECK(rmt_wait_tx_done(dshot->rmtChannel, 0));

    DShotRMT_setData(dshot, data);

    return rmt_write_items(dshot->rmtChannel, dshot->_dshotCmd, 17, wait);
}

esp_err_t DShotRMT_writePacket(DShotRMT_t *dshot, uint16_t payload, bool telemetry, bool wait) {
    uint16_t data = payload;

    data <<= 1;
    data |= telemetry;

    data = (data << 4) | DShotRMT_checksum(data);

    return DShotRMT_writeData(dshot, data, wait);
}

esp_err_t DShotRMT_repeatPacket(DShotRMT_t *dshot, uint16_t payload, bool telemetry, int n) {
    for (int i = 0; i < n; i++) {
        DSHOT_ERROR_CHECK(DShotRMT_writePacket(dshot, payload, telemetry, true));
        portYIELD();
    }
    return ESP_OK;
}

esp_err_t DShotRMT_repeatPacketTicks(DShotRMT_t *dshot, uint16_t payload, bool telemetry, TickType_t ticks) {
    DSHOT_ERROR_CHECK(rmt_wait_tx_done(dshot->rmtChannel, ticks));

    TickType_t repeatStop = xTaskGetTickCount() + ticks;
    while (xTaskGetTickCount() < repeatStop) {
        DSHOT_ERROR_CHECK(DShotRMT_writePacket(dshot, payload, telemetry, false));
        vTaskDelay(1);
    }
    return ESP_OK;
}
