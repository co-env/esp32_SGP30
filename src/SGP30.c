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

struct SGP30_dev dev;
static uint8_t dev_addr = SGP30_ADDR;



void sgp30_init(sgp30_t *sensor, sgp30_read_fptr_t user_i2c_read, sgp30_write_fptr_t user_i2c_write) {
    // i2c_master_driver_initialize();    
    // vTaskDelay(1000 / portTICK_RATE_MS);
    
    dev.intf_ptr = &dev_addr; 
    dev.i2c_read = user_i2c_read;
    dev.i2c_write = user_i2c_write;

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
    // if (sgp30_i2c_master_write(command, command_len) != ESP_OK) {
    if (dev.i2c_write(NULL_REG, command, command_len, dev.intf_ptr) != ESP_OK) {
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

    // if (sgp30_i2c_master_read(reply_buffer, reply_len) != ESP_OK) {
    if (dev.i2c_read(NULL_REG, reply_buffer, reply_len, dev.intf_ptr) != ESP_OK) {
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
