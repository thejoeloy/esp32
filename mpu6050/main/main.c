/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU6050 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "MPU6050";

#define I2C_MASTER_SCL_IO           26      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           25      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7

#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 9
#define FS_GYRO_2000 10

#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 9
#define FS_ACC_16G 10

#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107
#define REG_DATA_ACCEL_X 59
#define REG_DATA_ACCEL_Y 61
#define REG_DATA_ACCEL_Z 63
#define REG_DATA_GYR_X 67
#define REG_DATA_GYR_Y 69
#define REG_DATA_GYR_Z 71


/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

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

void mpu6050_init() {
    uint8_t data;
    esp_err_t ret;

    // 1. Check if the device is ready by reading the WHO_AM_I register
    ret = mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, &data, 1);
    if (ret != ESP_OK || data != 0x68) {  // 0x68 is the expected value for MPU6050
        ESP_LOGE(TAG, "MPU6050 not ready or invalid WHO_AM_I value");
        return;  // If the device is not ready, return early
    }
    ESP_LOGI(TAG, "MPU6050 is ready");

    // 2. Configure the Gyroscope (e.g., 500dps range)
    data = FS_GYRO_500;  // Set the gyro range to 500 dps
    ret = mpu6050_register_write_byte(REG_CONFIG_GYRO, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Gyroscope");
    } else {
        ESP_LOGI(TAG, "Gyroscope configured successfully");
    }

    // 3. Configure the Accelerometer (e.g., 4g range)
    data = FS_ACC_4G;  // Set the accelerometer range to 4g
    ret = mpu6050_register_write_byte(REG_CONFIG_ACC, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Accelerometer");
    } else {
        ESP_LOGI(TAG, "Accelerometer configured successfully");
    }

    // 4. Exit Sleep Mode
    data = 0;  // Clear the sleep bit
    ret = mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit sleep mode");
    } else {
        ESP_LOGI(TAG, "Exited Sleep Mode successfully");
    }
}

void mpu6050_read() {
    uint8_t data_x[2], data_y[2], data_z[2];
    uint8_t data_g_x[2], data_g_y[2], data_g_z[2];

    int16_t x_acc, y_acc, z_acc;
    int16_t x_gyr, y_gyr, z_gyr;

    // Read accelerometer data
    esp_err_t ret = mpu6050_register_read(REG_DATA_ACCEL_X, data_x, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading accelerometer X data");
        return;
    }
    ret = mpu6050_register_read(REG_DATA_ACCEL_Y, data_y, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading accelerometer Y data");
        return;
    }
    ret = mpu6050_register_read(REG_DATA_ACCEL_Z, data_z, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading accelerometer Z data");
        return;
    }

    // Read gyroscope data
    ret = mpu6050_register_read(REG_DATA_GYR_X, data_g_x, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading gyroscope X data");
        return;
    }
    ret = mpu6050_register_read(REG_DATA_GYR_Y, data_g_y, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading gyroscope Y data");
        return;
    }
    ret = mpu6050_register_read(REG_DATA_GYR_Z, data_g_z, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading gyroscope Z data");
        return;
    }

    // Convert byte data to 16-bit integers
    x_acc = ((int16_t)data_x[0] << 8) | data_x[1];
    y_acc = ((int16_t)data_y[0] << 8) | data_y[1];
    z_acc = ((int16_t)data_z[0] << 8) | data_z[1];

    x_gyr = ((int16_t)data_g_x[0] << 8) | data_g_x[1];
    y_gyr = ((int16_t)data_g_y[0] << 8) | data_g_y[1];
    z_gyr = ((int16_t)data_g_z[0] << 8) | data_g_z[1];

    // Print data to the log
    ESP_LOGI(TAG, "Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d", x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Reset MPU6050
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT));

	mpu6050_init();
    while (1)
    {
    	mpu6050_read();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
}
