/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU6050 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.  */
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "byteswap.h"

static const char *TAG = "i2c";

static void esp_warn_check(esp_err_t err, int line){
    if( err != ESP_OK ){
        ESP_LOGW(TAG, "Warning on line %d: %s", line, esp_err_to_name(err));
    }
}

#define ESP_WARN(err) esp_warn_check(err, __builtin_LINE()) 

#define I2C_MASTER_SCL_IO           19                        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                         /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                    0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR              0x75        /*!< Device ID */
/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t register_write(uint8_t reg_addr, uint8_t * data, size_t len){
    uint8_t * readback = (uint8_t *)malloc(len+1);
    readback[0] = reg_addr;
    for(size_t i = 0; i<len; ++i){ readback[i+1] = data[i];}
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, readback, len+1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if(ESP_OK == err){
        err = register_read(reg_addr, readback, len);
        if(ESP_OK == err){
            for(size_t i = 0; i < len; ++i){
                if(readback[i] != data[i])
                    err = ESP_ERR_INVALID_RESPONSE;
            }
        }
    }
    free(readback);
    return err;
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
static esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data)
{
    return register_write(reg_addr, &data, 1);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

struct xyz {
    int16_t x;
    int16_t y;
    int16_t z;
    float t;
};

#define ACCEL_LOG_SIZE 2000
static struct xyz accel_log[ACCEL_LOG_SIZE];
static volatile size_t accel_latest;

size_t accel_get_latest_idx(void)
{
    return accel_latest;
}

/* Binary packet format (all little-endian, matches ESP32 native byte order):
 *   [0..3]  float32  t0  — timestamp of first sample in seconds
 *   [4..5]  uint16   n   — number of samples
 *   [6+i*6 .. 6+i*6+5]  int16 x, int16 y, int16 z  for sample i
 */
size_t accel_copy_new_binary(size_t last_idx, uint8_t *buf, size_t buf_size)
{
    size_t latest = accel_latest;
    size_t n = (latest - last_idx + ACCEL_LOG_SIZE) % ACCEL_LOG_SIZE;
    if (n == 0 || buf_size < 12) {
        return 0;
    }
    size_t max_samples = (buf_size - 6) / 6;
    if (n > max_samples) {
        n = max_samples;
    }
    size_t first_idx = (latest - n + 1 + ACCEL_LOG_SIZE) % ACCEL_LOG_SIZE;
    float t0 = accel_log[first_idx].t;
    memcpy(buf, &t0, 4);
    uint16_t n16 = (uint16_t)n;
    memcpy(buf + 4, &n16, 2);
    size_t pos = 6;
    for (size_t i = 0; i < n; i++) {
        size_t idx = (first_idx + i) % ACCEL_LOG_SIZE;
        int16_t x = accel_log[idx].x;
        int16_t y = accel_log[idx].y;
        int16_t z = accel_log[idx].z;
        memcpy(buf + pos,     &x, 2);
        memcpy(buf + pos + 2, &y, 2);
        memcpy(buf + pos + 4, &z, 2);
        pos += 6;
    }
    return pos;
}

void accel_reader_task(void *pvParameters)
{
    uint8_t data[1024];
    uint64_t sample_period_us = 1000000ULL / 1000;  // 1000 Hz sample rate
    struct xyz accel;
    accel_latest = 0;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU6050 WHO_AM_I register, should match the i2c address */
    ESP_WARN(register_read(MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Configure the power control */
    data[0] = 4; // FIFO reset
    data[1] = 3; // Disable sleep. Use PLL from Z-axis gyroscope for more accurate clock
    data[2] = 0;
    ESP_WARN(register_write(0x6A, data, 3));

    /* Set the filter and sample rate */
    data[0] = 0;    // 1000 Hz sample rate
    data[1] = 0x21; // 100 Hz digital filter
    data[2] = 0;    // Gyro full-scale 250 deg/s
    data[3] = 0;    // Accel full-scale ±2G
    ESP_WARN(register_write(0x19, data, 4));

    ESP_WARN(register_write_byte(0x23, 0x08)); // Set only Accelerometer to fill FIFO
    ESP_WARN(register_write_byte(0x6A, 0x40)); // Enable FIFO
    while (1) {
        /* Check for FIFO overflow (INT_STATUS reg 0x3A, bit 4).
         * On overflow the FIFO is disabled and reads return 0x00, so
         * reset and re-enable it before reading any samples. */
        uint8_t int_status;
        ESP_WARN(register_read(0x3A, &int_status, 1));
        if (int_status & 0x10) {
            ESP_LOGW(TAG, "FIFO overflow — resetting FIFO");
            ESP_WARN(register_write_byte(0x6A, 0x04)); // FIFO reset
            ESP_WARN(register_write_byte(0x6A, 0x40)); // re-enable FIFO
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int16_t fifo_size;
        ESP_WARN(register_read(0x72, (uint8_t *)&fifo_size, 2));
        fifo_size = __bswap_16(fifo_size);
        if (fifo_size > (int16_t)sizeof(data)) {
            fifo_size = (int16_t)sizeof(data);
        }
        if (fifo_size >= 6) {
            uint64_t now_us = esp_timer_get_time();
            ESP_WARN(register_read(0x74, data, fifo_size));
            int n_new = fifo_size / 6;
            int16_t *buf = (int16_t *)data;
            for (int j = 0; j < n_new; j++) {
                accel.x = __bswap_16(*buf++);
                accel.y = __bswap_16(*buf++);
                accel.z = __bswap_16(*buf++);
                /* Back-extrapolate: sample j arrived (n_new-1-j) ms before now */
                accel.t = (float)(now_us - (uint64_t)(n_new - 1 - j) * sample_period_us) * 1e-6f;
                if (++accel_latest >= ACCEL_LOG_SIZE) {
                    accel_latest = 0;
                }
                accel_log[accel_latest] = accel;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_WARN(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
