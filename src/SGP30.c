/**
 * @file SGP30.c
 *
 * @author Renato Freitas 
 * 
 * @brief AS7262 library, based on Adafruit's one. 
 * ----> https://www.adafruit.com/products/3779
 * ----> https://github.com/adafruit/Adafruit_AS726x
 *
 * @date 10/2020
 */

#include "SGP30.h"

static const char *TAG = "SGP30-LIB";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

/***********************
 * I²C 16-bit Commands *
 ***********************/
static uint8_t INIT_AIR_QUALITY[2] =        { 0x20, 0x03 };
static uint8_t MEASURE_AIR_QUALITY[2] =     { 0x20, 0x08 };
static uint8_t GET_BASELINE[2] =            { 0x20, 0x15 };
static uint8_t SET_BASELINE[2] =            { 0x20, 0x1E };
static uint8_t SET_HUMIDITY[2] =            { 0x20, 0x61 };
static uint8_t MEASURE_TEST[2] =            { 0x20, 0x32 };
static uint8_t GET_FEATURE_SET_VERSION[2] = { 0x20, 0x2F };
static uint8_t MEASURE_RAW_SIGNALS[2] =     { 0x20, 0x50 };
static uint8_t GET_SERIAL_ID[2] =           { 0x36, 0x82 };
static uint8_t SOFT_RESET[2] =              { 0x00, 0x06 };


// ESP I2C Driver setup 
static esp_err_t i2c_master_driver_initialize(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    return i2c_param_config(i2c_master_port, &conf);
}

/** 
 * @brief Função genérica de leitura i2c de [size] bytes no [address]
 * 
 */

esp_err_t sgp30_i2c_master_read(uint8_t *data_read, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }

    uint8_t chip_addr = SGP30_ADDR;
    uint8_t len = size;

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_ERROR_CHECK(i2c_master_driver_initialize());

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        
    // Generates I2C START signal
    i2c_master_start(cmd);

    // Sends I2C READ header
    i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);

    // Reads incoming data
    if (len > 1) {
        i2c_master_read(cmd, data_read, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_read + len - 1, NACK_VAL);
    
    // Sends I2C STOP signal
    i2c_master_stop(cmd);


    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 2000 / portTICK_RATE_MS);


    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        // ESP_LOGI(TAG, "I2C Read Success: %02x", data_rd[0]);
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "I2C Read bus is busy");
    } else {
        ESP_LOGW(TAG, "I2C Read failed");
    }

    i2c_driver_delete(I2C_NUM_0);
    
    return 0;
}



/** 
 * @brief Função genérica de escrita i2c de [size] bytes no [address]
 * 
 */

esp_err_t sgp30_i2c_master_write(uint8_t *data_wr, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }

    uint8_t chip_addr = SGP30_ADDR;
    uint8_t len = size;

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_ERROR_CHECK(i2c_master_driver_initialize());


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    
    // Sends I2C START signal
    i2c_master_start(cmd);
    
    // Sends I2C WRITE header
    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);

    // Writes data to chip 
    for (int i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data_wr[i], ACK_CHECK_EN);
    }
    
    // Sends I2C STOP signal
    i2c_master_stop(cmd);


    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 2000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

     if (ret == ESP_OK) {
        // ESP_LOGI(TAG, "I2C Write Success: %02x", data_wr[0]);
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "I2C Read bus is busy");
    } else {
        ESP_LOGW(TAG, "I2C Read failed");
    }

    i2c_driver_delete(I2C_NUM_0);

    return 0;
}

void sgp30_init(sgp30_t *sensor) {
    sgp30_execute_command(GET_SERIAL_ID, 2, 10, sensor->serial_number, 3);
    ESP_LOGI(TAG, "%s - Serial Number: %02x %02x %02x", __FUNCTION__, sensor->serial_number[0],
                                sensor->serial_number[1], sensor->serial_number[2]);


    uint16_t featureset;
    // uint8_t command2[2] = { GET_FEATURE_SET_VERSION };
    sgp30_execute_command(GET_FEATURE_SET_VERSION, 2, 10, &featureset, 1);
    ESP_LOGI(TAG, "%s - Feature set version: %02x", __FUNCTION__, featureset);

    sgp30_IAQ_init(sensor);
}

void sgp30_softreset(sgp30_t *sensor) {
    sgp30_execute_command(SOFT_RESET, 2, 10, NULL, 0);
}

void sgp30_IAQ_init(sgp30_t *sensor) {
    sgp30_execute_command(INIT_AIR_QUALITY, 2, 10, NULL, 0);
}

void sgp30_IAQ_measure(sgp30_t *sensor) {
    uint16_t reply[2];

    sgp30_execute_command(MEASURE_AIR_QUALITY, 2, 20, reply, 2);
    sensor->TVOC = reply[1];
    sensor->eCO2 = reply[0];
}

void sgp30_IAQ_measure_raw(sgp30_t *sensor) {
    uint16_t reply[2];

    sgp30_execute_command(MEASURE_RAW_SIGNALS, 2, 20, reply, 2);
    sensor->raw_ethanol = reply[1];
    sensor->raw_H2 = reply[0];
}

void sgp30_get_IAQ_baseline(sgp30_t *sensor, uint16_t *eco2_base, uint16_t *tvoc_base) {
    uint16_t reply[2];

    sgp30_execute_command(GET_BASELINE, 2, 20, reply, 2);

    *eco2_base = reply[0];
    *tvoc_base = reply[1];
}

void sgp30_set_IAQ_baseline(sgp30_t *sensor, uint16_t eco2_base, uint16_t tvoc_base) {
    uint8_t baseline_command[8];

    baseline_command[0] = SET_BASELINE[0];
    baseline_command[1] = SET_BASELINE[1];

    baseline_command[2] = tvoc_base >> 8;
    baseline_command[3] = tvoc_base & 0xFF;
    baseline_command[4] = sgp30_calculate_CRC(baseline_command + 2, 2);

    baseline_command[5] = eco2_base >> 8;
    baseline_command[6] = eco2_base & 0xFF;
    baseline_command[7] = sgp30_calculate_CRC(baseline_command + 5, 2);

    sgp30_execute_command(baseline_command, 8, 20, NULL, 0);
}

void sgp30_set_humidity(sgp30_t *sensor, uint32_t absolute_humidity) {
    if (absolute_humidity > 256000) {
        ESP_LOGW(TAG, "%s - Abs humidity value %d is too high!", __FUNCTION__, absolute_humidity);
        return;
    }

    uint8_t ah_command[5];
    uint16_t ah_scaled = (uint16_t)(((uint64_t)absolute_humidity * 256 * 16777) >> 24);

    ah_command[0] = SET_HUMIDITY[0];
    ah_command[1] = SET_HUMIDITY[1];

    ah_command[2] = ah_scaled >> 8;
    ah_command[3] = ah_scaled & 0xFF;
    ah_command[4] = sgp30_calculate_CRC(ah_command + 2, 2);

    sgp30_execute_command(ah_command, 5, 20, NULL, 0);
}



//! @FIXME: CHANGE THIS TO ESP_ERR_T RETURN TYPE
bool sgp30_execute_command(uint8_t command[], uint8_t command_len, uint16_t delay, 
                            uint16_t *read_data, uint8_t read_len) {
    
    // Writes SGP30 Command
    if (sgp30_i2c_master_write(command, command_len) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write SGP30 I2C command!");
        return false;  
    }

    // Waits for chip to process command and measure desired value
    vTaskDelay(delay / portTICK_RATE_MS);

    // Checks if there is data to be read from the user, (or if it's just a simple command write)
    if (read_len == 0) {
        return true;
    }


    uint8_t reply_len = read_len * (SGP30_WORD_LEN + 1);
    uint8_t reply_buffer[reply_len];

    if (sgp30_i2c_master_read(reply_buffer, reply_len) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read SGP30 I2C command reply!");
        return false;  // failed to read reply buffer from chip
    }

    for (uint8_t i = 0; i < read_len; i++) {
        uint8_t crc = sgp30_calculate_CRC(reply_buffer + i * 3, 2);
        ESP_LOGD(TAG, "%s - Calc CRC: %02x,   Reply CRC: %02x", __FUNCTION__, crc, reply_buffer[i * 3 + 2]);

        if (crc != reply_buffer[i * 3 + 2]) {
            ESP_LOGW(TAG, "Reply and Calculated CRCs are different");
            return false;
        }

        // If CRCs are equal, save data
        read_data[i] = reply_buffer[i * 3];
        read_data[i] <<= 8;
        read_data[i] |= reply_buffer[i * 3 + 1];
        ESP_LOGD(TAG, "%s - Read data: %04x", __FUNCTION__, read_data[i]);
    }

    return true;
}

/**
 *  @brief Calculates 8-Bit checksum with given polynomial
 */
uint8_t sgp30_calculate_CRC(uint8_t *data, uint8_t len) {
    uint8_t crc = SGP30_CRC8_INIT;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
        if (crc & 0x80)
            crc = (crc << 1) ^ SGP30_CRC8_POLYNOMIAL;
        else
            crc <<= 1;
        }
    }
    return crc;
}
