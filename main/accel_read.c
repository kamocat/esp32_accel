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
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "byteswap.h"
#include "accel_read.h"

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

struct xyz{
    int16_t x;
    int16_t y;
    int16_t z;
};

void accel_reader_task(void *pvParameters)
{
    struct chunk_t chunk;
    uint8_t data[300];
    int16_t * buf = (int16_t *) data;
    QueueHandle_t queue = (QueueHandle_t) pvParameters;
    size_t q_size = uxQueueSpacesAvailable(queue);
    struct accel_t ** accel = malloc((sizeof(data)+sizeof(void*))* q_size);
    accel[0] = accel + q_size;
    for(size_t i = 1; i < q_size; ++i){
        accel[i] = accel[i-1] + sizeof(data);
    }
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU6050 WHO_AM_I register, should match the i2c addres */
    ESP_WARN(register_read(MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Configure the power control */
    data[0] = 4; // FIFO reset
    data[1] = 3; // Disable sleep. Use PLL from Z-axis gyroscope for more accurate clock
    data[2] = 0;
    ESP_WARN(register_write(0x6A, data, 3));

    /* Set the filter and sample rate */
    data[0] = 9; // 100Hz sample rate
    data[1] = 0x25; // 10Hz digital filter
    data[2] = 0; // Disable Gyro self-test, set full-scale to 250 degrees/second
    data[3] = 0; // Disable accel self-test, set full scale to 2G
    ESP_WARN(register_write(0x19, data, 4));

    ESP_WARN(register_write_byte(0x23,0x08)); // Set only Accelerometer to fill FIFO
    ESP_WARN(register_write_byte(0x6A, 0x40));// Enable FIFO
    while(1){for(size_t i = 0; i < q_size; ++i){
        int16_t fifo_size;
        ESP_WARN(register_read(0x72,(uint8_t*)&fifo_size,2));
        fifo_size = __bswap_16(fifo_size);
        ESP_LOGI(TAG, "Fifo size: %d", fifo_size);
        if(fifo_size > sizeof(data))
            fifo_size = sizeof(data);
        ESP_WARN(register_read(0x74,(uint8_t*)buf,fifo_size));
        chunk.len = 0;
        chunk.data = accel[i];
        for(;fifo_size>0;fifo_size -=6){
            chunk.data[chunk.len].x = __bswap_16(*buf++);
            chunk.data[chunk.len].y = __bswap_16(*buf++);
            chunk.data[chunk.len].z = __bswap_16(*buf++);
            ++chunk.len;
        }
        xQueueSend(queue, (void*)&chunk, 200 / portTICK_PERIOD_MS);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }}

    ESP_WARN(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
