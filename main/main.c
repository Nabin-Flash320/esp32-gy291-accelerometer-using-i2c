

#include "trace.h"
#include "driver/i2c.h"
#include "main_defs.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"


#define GY291_SDA 2
#define GY291_SCL 3
#define GY291_I2C_NUM I2C_NUM_0

static const i2c_config_t gy291_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GY291_SDA,
    .scl_io_num = GY291_SCL,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
};

static float find_square_root(float x, float y, float z)
{
    return sqrtf(pow(x, 2) + pow(y, 2) + pow(z, 2));
}


esp_err_t write_reg(uint8_t data, uint8_t register_addr)
{
    esp_err_t error = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(cmd));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(cmd, register_addr, ACK_CHECK_EN));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(cmd, data, ACK_CHECK_EN));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_stop(cmd));
    i2c_master_cmd_begin(GY291_I2C_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return error;
}


esp_err_t read_reg(uint8_t *data, uint8_t register_addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(cmd));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(cmd, register_addr, ACK_CHECK_EN));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(cmd));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_read_byte(cmd, data, I2C_MASTER_NACK));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_stop(cmd));
    i2c_master_cmd_begin(GY291_I2C_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}


static uint8_t get_device_id()
{
    uint8_t dev_id = 0;
    read_reg(&dev_id, ADXL345_DEVICE_ID_REGISTER);
    return dev_id;
}


static uint8_t data_formatting()
{
    uint8_t data_format = ADXL345_FORMAT_REGISTER_DATA;
    uint8_t format = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(data_format, ADXL345_DATA_FORMAT_REGISTER));
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(&format, ADXL345_DATA_FORMAT_REGISTER));
    return format;
}


static uint8_t get_data_rate()
{
    uint8_t data_rate = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(read_reg(&data_rate, ADXL345_DATA_RATE));
    return data_rate;
}

static bool set_to_measure_mode()
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(ADXL345_POWER_CTRL_SET_TO_MEASUTEMENT, ADXL345_DEVICE_POWER_CTRL));
    return ESP_OK;
}

static bool reset_to_measure_mode()
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(ADXL345_POWER_CTRL_RESET, ADXL345_DEVICE_POWER_CTRL));
    return ESP_OK;
}

static int16_t get_x_axis_value()
{
    uint8_t buffer_0, buffer_1;
    read_reg(&buffer_0, ADXL345_DATA_X_0_REGISTER);
    read_reg(&buffer_1, ADXL345_DATA_X_1_REGISTER);
    // TRACE_B("x-axis: (buffer_0, buffer_1) = (%x, %x)", buffer_0, buffer_1);
    int16_t x_data = (buffer_1 << 8) | buffer_0;
    // TRACE_B("x_data is %d", x_data);
    return x_data;
}

static int16_t get_y_axis_value()
{
    uint8_t buffer_0, buffer_1;
    read_reg(&buffer_0, ADXL345_DATA_Y_0_REGISTER);
    read_reg(&buffer_1, ADXL345_DATA_Y_1_REGISTER);
    // TRACE_B("y-axis: (buffer_0, buffer_1) = (%x, %x)", buffer_0, buffer_1);
    int16_t y_data = (buffer_1 << 8) | buffer_0;
    // TRACE_B("y_data is %d", y_data);
    return y_data;
}

static int16_t get_z_axis_value()
{
    uint8_t buffer_0, buffer_1;
    read_reg(&buffer_0, ADXL345_DATA_Z_0_REGISTER);
    read_reg(&buffer_1, ADXL345_DATA_Z_1_REGISTER);
    // TRACE_B("z-axis: (buffer_0, buffer_1) = (%x, %x)", buffer_0, buffer_1);
    int16_t z_data = (buffer_1 << 8) | buffer_0;
    
    return z_data;
}

static void get_axis_value(void* parms)
{   
    float x_data, y_data, z_data;
    // Reset power control register before configuring the device.
    ESP_ERROR_CHECK_WITHOUT_ABORT(reset_to_measure_mode());

    TRACE_E("The data format is %x", (data_formatting() & 0x03));
    TRACE_E("The data rate is %x", get_data_rate());
    TRACE_E("The device id is %x", get_device_id());

    ESP_ERROR_CHECK_WITHOUT_ABORT(set_to_measure_mode());
    
    while(1)
    {   
        TRACE_D("<++++++++++++++++++++++++++++++++++++++>");
        x_data = (get_x_axis_value() * ADXL345_CONVERTER_FACTOR_MG_TO_G * STANDARD_G_TO_ACCEL_CONVERSION_VALUE) / 8.1;
        y_data = (get_y_axis_value() * ADXL345_CONVERTER_FACTOR_MG_TO_G * STANDARD_G_TO_ACCEL_CONVERSION_VALUE) / 8.1;
        z_data = (get_z_axis_value() * ADXL345_CONVERTER_FACTOR_MG_TO_G * STANDARD_G_TO_ACCEL_CONVERSION_VALUE) / 8.1;
        TRACE_B("(x, y, z) = (%f, %f, %f)", x_data, y_data, z_data);
        TRACE_B("Root is %f", find_square_root(x_data, y_data, z_data));
        TRACE_D("<++++++++++++++++++++++++++++++++++++++>");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    esp_err_t error;


    TRACE_B("GY291-i2c");
    error = i2c_param_config(GY291_I2C_NUM, &gy291_config);
    if(ESP_OK != error)
    {
        TRACE_E("Error confugring I2C params.(code:%s)", esp_err_to_name(error));
    }
    else
    {
        TRACE_I("I2C params confiugred successfully!");
    }

    error = i2c_driver_install(GY291_I2C_NUM, gy291_config.mode, 0, 0, 0);
    if(ESP_OK != error)
    {
        TRACE_E("Error installing I2C driver.(code:%s)", esp_err_to_name(error));
    }
    else
    {
        TRACE_I("I2C driver installed successfully!");
    }
    
    xTaskCreate(get_axis_value, "get_axis_value", 2*2048, NULL, 0, NULL);
    
}

