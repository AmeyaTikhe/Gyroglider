#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"
#include "driver/rmt_tx.h"
#include "driver/adc.h"
#include "dshot_esc_encoder.h"
#include <time.h> // Include time.h for FreeRTOS ticks

float setpoint = 25;
// float kp = 12;
// float kd = 0;
// float ki = 0;
int baseValue = 460;
float integralError = 0.0;
float prevError = 0.0;

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_ESC_RESOLUTION_HZ 32000000 // 32MHz resolution, DSHot protocol needs a relative high resolution
#else
#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#endif
#define DSHOT_ESC_GPIO_NUM_1 2
#define DSHOT_ESC_GPIO_NUM_2 15

static const char *TAG = "SemiDroneMode";

// CUSTOM FUNCTIONSd
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int pid_calc(float setpoint, float currentAngle, float kp, float kd, float ki, int baseValue, float *prevError, float *integralError)
{
    float error = setpoint - currentAngle;
    *integralError += error;
    float P_term = (kp * error);
    float D_term = (kd * (error - *prevError));
    float I_term = (ki * (*integralError));

    int throttle = baseValue + (int)(P_term + D_term + I_term);

    *prevError = error;
    return throttle;
}

float bound(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}
// Function to get the current time in milliseconds using FreeRTOS tick count
uint64_t get_time_in_ms()
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void self_balance(void *arg)
{
    adc1_config_width(ADC_WIDTH_BIT_12);

    rmt_channel_handle_t esc_chan1 = NULL;
    rmt_channel_handle_t esc_chan2 = NULL;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ESP_LOGI(TAG, "Create First RMT TX channel");

    rmt_tx_channel_config_t tx_chan_config1 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = DSHOT_ESC_GPIO_NUM_1,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config1, &esc_chan1));

    ESP_LOGI(TAG, "Create Second RMT TX channel");

    rmt_tx_channel_config_t tx_chan_config2 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = DSHOT_ESC_GPIO_NUM_2,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config2, &esc_chan2));

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ESP_LOGI(TAG, "Install Dshot ESC encoder");
    rmt_encoder_handle_t dshot_encoder1 = NULL;
    rmt_encoder_handle_t dshot_encoder2 = NULL;

    dshot_esc_encoder_config_t encoder_config1 = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config1, &dshot_encoder1));

    dshot_esc_encoder_config_t encoder_config2 = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config2, &dshot_encoder2));

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(esc_chan1));
    ESP_ERROR_CHECK(rmt_enable(esc_chan2));

    rmt_transmit_config_t tx_config1 = {
        .loop_count = -1,
    };
    rmt_transmit_config_t tx_config2 = {
        .loop_count = -1,
    };

    dshot_esc_throttle_t throttle1 = {
        .throttle = 0,
        .telemetry_req = false,
    };
    dshot_esc_throttle_t throttle2 = {
        .throttle = 0,
        .telemetry_req = false,
    };

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ESP_LOGI(TAG, "Start ESC by sending zero throttle for a while...");
    ESP_ERROR_CHECK(rmt_transmit(esc_chan1, dshot_encoder1, &throttle1, sizeof(throttle1), &tx_config1));
    ESP_ERROR_CHECK(rmt_transmit(esc_chan2, dshot_encoder2, &throttle2, sizeof(throttle2), &tx_config2));

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Increase throttle, no telemetry");

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while (true)
    {
        float euler_angle[2], mpu_offset[2] = {0.0f, 0.0f};
        int thro = 0;

        if (enable_mpu6050() == ESP_OK)
        {

            while (true)
            {
                read_mpu6050(euler_angle, mpu_offset);

                float currentAngle = euler_angle[1];
                int finalThro = (int) read_pid_const().kp;


                ESP_LOGI(TAG, "Throttle: %d | Pitch: %0.2f", finalThro, euler_angle[1]);

                if (read_pid_const().val_changed)
                {
                    ESP_LOGE(TAG, "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);

                    reset_val_changed_pid_const();
                }

                throttle1.throttle = finalThro;
                throttle2.throttle = finalThro;

                ESP_ERROR_CHECK(rmt_transmit(esc_chan1, dshot_encoder1, &throttle1, sizeof(throttle1), &tx_config1));
                ESP_ERROR_CHECK(rmt_transmit(esc_chan2, dshot_encoder2, &throttle2, sizeof(throttle2), &tx_config2));

                ESP_ERROR_CHECK(rmt_disable(esc_chan1));
                ESP_ERROR_CHECK(rmt_enable(esc_chan1));
                ESP_ERROR_CHECK(rmt_disable(esc_chan2));
                ESP_ERROR_CHECK(rmt_enable(esc_chan2));
            }
        }

        ESP_LOGE(TAG, "MPU Initialisation Failed / Connection Broke!");
    }

    vTaskDelete(NULL);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


void app_main()
{
    xTaskCreate(&self_balance, "self_balance", 4096, NULL, 1, NULL);

    start_tuning_http_server();
}