/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a ADXL345 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.  */
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_timer.h"

static const char *TAG = "i2c";

static void esp_warn_check(esp_err_t err, int line){
    if( err != ESP_OK ){
        ESP_LOGW(TAG, "Warning on line %d: %s", line, esp_err_to_name(err));
    }
}

#define ESP_WARN(err) esp_warn_check(err, __builtin_LINE()) 

#define I2C_MASTER_SCL_IO           19                        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           18                        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                         /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define ADXL345_SENSOR_ADDR                    0x53        /*!< Slave address of the ADXL345 sensor */
#define ADXL345_WHO_AM_I_REG_ADDR              0xE5        /*!< Device ID */
/**
 * @brief Read a sequence of bytes from a ADXL345 sensor registers
 */
static esp_err_t adxl_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, ADXL345_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a ADXL345 sensor register
 */
static esp_err_t adxl_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
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
    float x;
    float y;
    float z;
    float t;
};

static float unpack_bytes(int8_t lower_byte, int16_t upper_byte){
    float x  = (upper_byte<<8)|lower_byte;
    x /= 256;
    return x;
}

static esp_err_t adxl_get_accel(struct xyz * data){
    int8_t buf[6];
    esp_err_t err = adxl_register_read(0x32, (uint8_t*)buf, 6);
    //ESP_LOG_BUFFER_HEX(TAG, buf, 6);
    data->x = unpack_bytes(buf[0], buf[1]);
    data->y = unpack_bytes(buf[2], buf[3]);
    data->z = unpack_bytes(buf[4], buf[5]);
    data->t = esp_timer_get_time() * 0.000001;
    return err;
}

#define ACCEL_LOG_SIZE 30
static struct xyz accel_log[ACCEL_LOG_SIZE];
static size_t accel_latest;

#define LAMBDA_COPY(val, stop) for(;i<stop;++i){written+=snprintf(dest+written,size-written,fmt,val);if((size-written)<min_size){return written;}}

size_t accel_writer(char * dest, size_t size){
    const size_t min_size = 20; //Stop writing if not enough bytes are left
    const char * fmt = "%.3f,"; //Format floats to 3 digits past the decimal
    size_t written = 0;

    // All persistent variables are declared static
    static size_t i1, i2, i3, i4, i;
    static int iter = 0;
    switch(iter){ // Used to maintain state. Fall-through intended.
        case 0: // Setup
            i1 = accel_latest + 5;
            i2 = ACCEL_LOG_SIZE;
            i3 = 0;
            i4 = accel_latest;
            i = i1;
            written = snprintf(dest, size, "[["); // Start of JSON array
            __attribute__ ((fallthrough));
        case 1:
            iter = 1;
            LAMBDA_COPY(accel_log[i].t, i2)
            i = i3;
            __attribute__ ((fallthrough));
        case 2:
            iter = 2;
            LAMBDA_COPY(accel_log[i].t, i4)
            written += snprintf(dest+written, size-written, "],["); // Array seperator
            i = i1;
            __attribute__ ((fallthrough));
        case 3:
            iter = 3;
            LAMBDA_COPY(accel_log[i].x, i2)
            i = i3;
            __attribute__ ((fallthrough));
        case 4:
            iter = 4;
            LAMBDA_COPY(accel_log[i].x, i4)
            written += snprintf(dest+written, size-written, "],["); // Array seperator
            i = i1;
            __attribute__ ((fallthrough));
        case 5:
            iter = 5;
            LAMBDA_COPY(accel_log[i].y, i2)
            i = i3;
            __attribute__ ((fallthrough));
        case 6:
            iter = 6;
            LAMBDA_COPY(accel_log[i].y, i4)
            written += snprintf(dest+written, size-written, "],["); // Array seperator
            i = i1;
            __attribute__ ((fallthrough));
        case 7:
            iter = 7;
            LAMBDA_COPY(accel_log[i].z, i2)
            i = i3;
            __attribute__ ((fallthrough));
        case 8:
            iter = 8;
            LAMBDA_COPY(accel_log[i].z, i4)
            written += snprintf(dest+written, size-written, "]]"); // Array end
            iter = 9;
            return written;
        default:
            iter = 0;
            return 0; // Finished writing
    }
}

void accel_reader_task(void *pvParameters)
{
    uint8_t data[2];
    struct xyz accel;
    accel_latest = 0;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the ADXL345 WHO_AM_I register, on power up the register should have the value 0xE5 */
    ESP_WARN(adxl_register_read(ADXL345_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Configure the power control */
    ESP_WARN(adxl_register_write_byte(0x2D, 0x08));

    /* Configure data scaling */
    ESP_WARN(adxl_register_write_byte(0x31, 0x0B));

    while(1){
        ESP_WARN(adxl_get_accel(&accel));
        if( ++accel_latest >= ACCEL_LOG_SIZE)
            accel_latest = 0;
        accel_log[accel_latest] = accel;
        ESP_LOGI(TAG, "%.3f %.3f %.3f %.3f %d", accel.x, accel.y, accel.z, accel.t, accel_latest);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ESP_WARN(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
