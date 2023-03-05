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
    float t;
};

static esp_err_t get_accel(struct xyz * data){
    int16_t buf[3];
    esp_err_t err = register_read(0x3B, (uint8_t*)buf, 6);
    //sESP_LOG_BUFFER_HEX(TAG, buf, 6);
    data->x = __bswap_16(buf[0]);
    data->y = __bswap_16(buf[1]);
    data->z = __bswap_16(buf[2]);
    data->t = esp_timer_get_time() * 0.000001;
    return err;
}

#define ACCEL_LOG_SIZE 400
static struct xyz accel_log[ACCEL_LOG_SIZE];
static size_t accel_latest;

#define LAMBDA_COPY(val, stop, fmt) for(;i<stop;++i){written+=snprintf(dest+written,size-written,fmt,val);if((size-written)<min_size){return written;}}

size_t accel_writer(char * dest, size_t size){
    const size_t min_size = 20; //Stop writing if not enough bytes are left
    const char * fmt = "%.3f,"; //Format floats to 3 digits past the decimal
    const char * dfmt = "%d,"; //Integer format
    size_t written = 0;

    // All persistent variables are declared static
    static size_t i1, i2, i3, i4, i;
    static int iter = 0;
    switch(iter){ // Used to maintain state. Fall-through intended.
        case 0: // Setup
            i1 = accel_latest + 10;
            i2 = ACCEL_LOG_SIZE;
            i3 = 0;
            i4 = accel_latest+1;
            i = i1;
            written = snprintf(dest, size, "[["); // Start of JSON array
            __attribute__ ((fallthrough));
        case 1:
            iter = 1;
            LAMBDA_COPY(accel_log[i].t, i2, fmt)
            i = i3;
            __attribute__ ((fallthrough));
        case 2:
            iter = 2;
            LAMBDA_COPY(accel_log[i].t, i4, fmt)
            --written;
            written += snprintf(dest+written, size-written, "],["); // Array seperator
            i = i1;
            __attribute__ ((fallthrough));
        case 3:
            iter = 3;
            LAMBDA_COPY(accel_log[i].x, i2, dfmt)
            i = i3;
            __attribute__ ((fallthrough));
        case 4:
            iter = 4;
            LAMBDA_COPY(accel_log[i].x, i4, dfmt)
            --written;
            written += snprintf(dest+written, size-written, "],["); // Array seperator
            i = i1;
            __attribute__ ((fallthrough));
        case 5:
            iter = 5;
            LAMBDA_COPY(accel_log[i].y, i2, dfmt)
            i = i3;
            __attribute__ ((fallthrough));
        case 6:
            iter = 6;
            LAMBDA_COPY(accel_log[i].y, i4, dfmt)
            --written;
            written += snprintf(dest+written, size-written, "],["); // Array seperator
            i = i1;
            __attribute__ ((fallthrough));
        case 7:
            iter = 7;
            LAMBDA_COPY(accel_log[i].z, i2, dfmt)
            i = i3;
            __attribute__ ((fallthrough));
        case 8:
            iter = 8;
            LAMBDA_COPY(accel_log[i].z, i4, dfmt)
            --written;
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
    uint8_t data[128];
    struct xyz accel;
    accel_latest = 0;
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
    accel.t = 0;
    while(1){
        //ESP_WARN(get_accel(&accel));
        int16_t fifo_size;
        ESP_WARN(register_read(0x72,(uint8_t*)&fifo_size,2));
        fifo_size = __bswap_16(fifo_size);
        ESP_LOGI(TAG, "Fifo size: %d", fifo_size);
        if(fifo_size > sizeof(data))
            fifo_size = sizeof(data);
        ESP_WARN(register_read(0x74,data,fifo_size));
        int16_t * buf = (int16_t *)data;
        for(;fifo_size>0;fifo_size -=6){
            accel.x = __bswap_16(*buf++);
            accel.y = __bswap_16(*buf++);
            accel.z = __bswap_16(*buf++);
            accel.t+= 0.01;
            if( ++accel_latest >= ACCEL_LOG_SIZE)
                accel_latest = 0;
            accel_log[accel_latest] = accel;
        }
        //ESP_LOGI(TAG, "%d %d %d", accel.x, accel.y, accel.z);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_WARN(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
