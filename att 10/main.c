#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_log.h"
#include "esp_err.h"

#define POT_GPIO GPIO_NUM_4
#define LED_GPIO GPIO_NUM_18
#define BUTTON_GPIO GPIO_NUM_5

#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9

#define MPU6050_ADDRESS 0x68

#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION LEDC_TIMER_13_BIT
#define PWM_MAX_DUTY 8191

#define ADC_MAX_VALUE 4095

#define POT_SAMPLE_TIME pdMS_TO_TICKS(50)
#define IMU_SAMPLE_TIME pdMS_TO_TICKS(100)
#define CONSOLE_SAMPLE_TIME pdMS_TO_TICKS(500)

static const char *TAG = "MULTITASK_SYSTEM";

typedef enum {
    MODE_LIVE,
    MODE_HOLD
} system_mode_t;

typedef struct {
    uint16_t pot_raw;
    int pot_mv;
    uint32_t led_duty;
} pot_data_t;

typedef struct {
    system_mode_t mode;
    uint16_t pot_raw;
    int pot_mv;
    uint32_t led_duty;
    float accel_x;
    float accel_y;
    float accel_z;
} telemetry_t;

static QueueHandle_t pot_queue;
static SemaphoreHandle_t button_semaphore;
static SemaphoreHandle_t telemetry_mutex;

static telemetry_t telemetry = {
    .mode = MODE_LIVE,
    .pot_raw = 0,
    .pot_mv = 0,
    .led_duty = 0,
    .accel_x = 0.0f,
    .accel_y = 0.0f,
    .accel_z = 0.0f
};

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mpu_handle;

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &adc_config,
            &adc_handle
        )
    );

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            ADC_CHANNEL_3,
            &channel_config
        )
    );

    adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_3,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    ESP_ERROR_CHECK(
        adc_cali_create_scheme_curve_fitting(
            &calibration_config,
            &adc_cali_handle
        )
    );
}

static void init_led(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(
        ledc_timer_config(&timer_config)
    );

    ledc_channel_config_t channel_config = {
        .gpio_num = LED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&channel_config)
    );
}

static void init_button(void)
{
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(
        gpio_config(&button_config)
    );
}

static void init_i2c(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = false
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_config,
            &i2c_bus
        )
    );

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDRESS,
        .scl_speed_hz = 100000
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            i2c_bus,
            &device_config,
            &mpu_handle
        )
    );
}

static esp_err_t mpu_write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_transmit(
        mpu_handle,
        data,
        sizeof(data),
        1000
    );
}

static esp_err_t mpu_read_acceleration(
    float *x,
    float *y,
    float *z
)
{
    uint8_t reg = 0x3B;
    uint8_t data[6];

    esp_err_t result = i2c_master_transmit_receive(
        mpu_handle,
        &reg,
        1,
        data,
        sizeof(data),
        1000
    );

    if (result != ESP_OK) {
        return result;
    }

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    *x = (float)raw_x / 16384.0f;
    *y = (float)raw_y / 16384.0f;
    *z = (float)raw_z / 16384.0f;

    return ESP_OK;
}

static void init_mpu6050(void)
{
    ESP_ERROR_CHECK(
        mpu_write_register(0x6B, 0x00)
    );

    ESP_ERROR_CHECK(
        mpu_write_register(0x1C, 0x00)
    );
}

static void button_task(void *parameter)
{
    int previous_state = 1;

    while (1) {

        int current_state = gpio_get_level(BUTTON_GPIO);

        if (previous_state == 1 && current_state == 0) {

            vTaskDelay(pdMS_TO_TICKS(50));

            if (gpio_get_level(BUTTON_GPIO) == 0) {

                xSemaphoreGive(button_semaphore);

                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        previous_state = gpio_get_level(BUTTON_GPIO);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void mode_task(void *parameter)
{
    while (1) {

        if (xSemaphoreTake(
                button_semaphore,
                portMAX_DELAY
            ) == pdTRUE) {

            xSemaphoreTake(
                telemetry_mutex,
                portMAX_DELAY
            );

            if (telemetry.mode == MODE_LIVE) {
                telemetry.mode = MODE_HOLD;
            } else {
                telemetry.mode = MODE_LIVE;
            }

            system_mode_t current_mode = telemetry.mode;

            xSemaphoreGive(telemetry_mutex);

            ESP_LOGI(
                TAG,
                "Mode changed: %s",
                current_mode == MODE_LIVE ? "LIVE" : "HOLD"
            );
        }
    }
}

static void pot_task(void *parameter)
{
    pot_data_t pot_data;

    while (1) {

        int raw_value = 0;
        int voltage_mv = 0;

        ESP_ERROR_CHECK(
            adc_oneshot_read(
                adc_handle,
                ADC_CHANNEL_3,
                &raw_value
            )
        );

        ESP_ERROR_CHECK(
            adc_cali_raw_to_voltage(
                adc_cali_handle,
                raw_value,
                &voltage_mv
            )
        );

        pot_data.pot_raw = raw_value;
        pot_data.pot_mv = voltage_mv;

        pot_data.led_duty =
            ((uint32_t)raw_value * PWM_MAX_DUTY)
            / ADC_MAX_VALUE;

        xQueueOverwrite(
            pot_queue,
            &pot_data
        );

        vTaskDelay(POT_SAMPLE_TIME);
    }
}

static void led_task(void *parameter)
{
    pot_data_t pot_data;

    while (1) {

        if (xQueueReceive(
                pot_queue,
                &pot_data,
                portMAX_DELAY
            ) == pdTRUE) {

            xSemaphoreTake(
                telemetry_mutex,
                portMAX_DELAY
            );

            telemetry.pot_raw = pot_data.pot_raw;
            telemetry.pot_mv = pot_data.pot_mv;

            if (telemetry.mode == MODE_LIVE) {

                telemetry.led_duty = pot_data.led_duty;

                ESP_ERROR_CHECK(
                    ledc_set_duty(
                        LEDC_LOW_SPEED_MODE,
                        LEDC_CHANNEL_0,
                        pot_data.led_duty
                    )
                );

                ESP_ERROR_CHECK(
                    ledc_update_duty(
                        LEDC_LOW_SPEED_MODE,
                        LEDC_CHANNEL_0
                    )
                );
            }

            xSemaphoreGive(telemetry_mutex);
        }
    }
}

static void imu_task(void *parameter)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {

        float x, y, z;

        esp_err_t result = mpu_read_acceleration(
            &x,
            &y,
            &z
        );

        if (result == ESP_OK) {

            xSemaphoreTake(
                telemetry_mutex,
                portMAX_DELAY
            );

            telemetry.accel_x = x;
            telemetry.accel_y = y;
            telemetry.accel_z = z;

            xSemaphoreGive(telemetry_mutex);

        } else {

            ESP_LOGE(
                TAG,
                "MPU6050 read error: %s",
                esp_err_to_name(result)
            );
        }

        vTaskDelayUntil(
            &last_wake_time,
            IMU_SAMPLE_TIME
        );
    }
}

static void console_task(void *parameter)
{
    telemetry_t snapshot;

    while (1) {

        xSemaphoreTake(
            telemetry_mutex,
            portMAX_DELAY
        );

        snapshot = telemetry;

        xSemaphoreGive(telemetry_mutex);

        uint32_t led_percentage =
            (snapshot.led_duty * 100) / PWM_MAX_DUTY;

        printf("\n");
        printf("=====================================================\n");

        printf(
            "STATUS: [%s] | POT: %u (%d mV) | LED: %lu%%\n",
            snapshot.mode == MODE_LIVE ? "LIVE" : "HOLD",
            snapshot.pot_raw,
            snapshot.pot_mv,
            (unsigned long)led_percentage
        );

        printf(
            "IMU ACCEL (g): X: %.2f | Y: %.2f | Z: %.2f\n",
            snapshot.accel_x,
            snapshot.accel_y,
            snapshot.accel_z
        );

        printf("=====================================================\n");

        vTaskDelay(CONSOLE_SAMPLE_TIME);
    }
}

void app_main(void)
{
    init_adc();
    init_led();
    init_button();
    init_i2c();
    init_mpu6050();

    pot_queue = xQueueCreate(
        1,
        sizeof(pot_data_t)
    );

    button_semaphore = xSemaphoreCreateBinary();

    telemetry_mutex = xSemaphoreCreateMutex();

    if (
        pot_queue == NULL ||
        button_semaphore == NULL ||
        telemetry_mutex == NULL
    ) {
        ESP_LOGE(
            TAG,
            "Failed to create RTOS IPC objects"
        );

        abort();
    }

    xTaskCreate(
        button_task,
        "button_task",
        2048,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        mode_task,
        "mode_task",
        2048,
        NULL,
        4,
        NULL
    );

    xTaskCreate(
        pot_task,
        "pot_task",
        3072,
        NULL,
        3,
        NULL
    );

    xTaskCreate(
        led_task,
        "led_task",
        3072,
        NULL,
        3,
        NULL
    );

    xTaskCreate(
        imu_task,
        "imu_task",
        3072,
        NULL,
        4,
        NULL
    );

    xTaskCreate(
        console_task,
        "console_task",
        3072,
        NULL,
        1,
        NULL
    );

    ESP_LOGI(
        TAG,
        "Multitask system initialized"
    );
}