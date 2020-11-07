/**
 * @file SGP30.h
 *
 * @author Renato Freitas 
 * 
 * @brief SGP30 library, based on Adafruit's one. 
 * ----> https://www.adafruit.com/product/3709
 * ----> https://github.com/adafruit/Adafruit_SGP30
 *
 * @date 10/2020
 */

#ifndef __SGP30_H__
#define __SGP30_H__

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <string.h>

/***********************************
    Default I2C address
************************************/
#define SGP30_ADDR  (0x58) 


/****** Commands and constants ******/
#define SGP30_FEATURESET 0x0020    /**< The required set for this library */
#define SGP30_CRC8_POLYNOMIAL 0x31 /**< Seed for SGP30's CRC polynomial */
#define SGP30_CRC8_INIT 0xFF       /**< Init value for CRC */
#define SGP30_WORD_LEN 2           /**< 2 bytes per word */

//!!!
#define NULL_REG 0xFF //????? 
//!!!

/** SGP30 Main Data Struct */
typedef struct {
    /**< The 48-bit serial number, this value is set when you call {@link begin()} **/
    uint16_t serial_number[3];

    /**< The last measurement of the IAQ-calculated Total Volatile Organic
            Compounds in ppb. This value is set when you call {@link IAQmeasure()} **/
    uint16_t TVOC;
     
    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This
            value is set when you call {@link IAQmeasure()} **/
    uint16_t eCO2;

    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This
            value is set when you call {@link IAQmeasureRaw()} **/
    uint16_t raw_H2;

    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This 
            value is set when you call {@link IAQmeasureRaw()} **/
    uint16_t raw_ethanol;

} sgp30_t;

/*** Function Pointers ***/
typedef int8_t (*sgp30_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
typedef int8_t (*sgp30_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
struct SGP30_dev {
    /*< Interface function pointer used to enable the device address for I2C and chip selection for SPI */
    void *intf_ptr;

    /*< Read function pointer */
    sgp30_read_fptr_t i2c_read;

    /*< Write function pointer */
    sgp30_write_fptr_t i2c_write;
};


/**
 *  @brief  Setups the hardware and detects a valid SGP30. Initializes I2C
 *          then reads the serialnumber and checks that we are talking to an SGP30.
 */
void sgp30_init(sgp30_t *sensor, sgp30_read_fptr_t user_i2c_read, sgp30_write_fptr_t user_i2c_write);

/**
 *   @brief  Commands the sensor to perform a soft reset using the "General
 *           Call" mode. Take note that this is not sensor specific and all devices that
 *           support the General Call mode on the on the same I2C bus will perform this.
 */
void sgp30_softreset(sgp30_t *sensor);

/**
 *   @brief  Commands the sensor to begin the IAQ algorithm. Must be called
 *           after startup.
 */
void sgp30_IAQ_init(sgp30_t *sensor);

/**
 *  @brief  Commands the sensor to take a single eCO2/VOC measurement. Places
 *          results in sensor.TVOC and sensor.eCO2
 */

void sgp30_IAQ_measure(sgp30_t *sensor);

/**
 *  @brief  Commands the sensor to take a single H2/ethanol raw measurement.
 *          Places results in sensor.raw_H2 and sensor.raw_ethanol
 */
void sgp30_IAQ_measure_raw(sgp30_t *sensor);

/*!
 *   @brief  Request baseline calibration values for both CO2 and TVOC IAQ
 *           calculations. Places results in parameter memory locaitons.
 *   @param  eco2_base
 *           A pointer to a uint16_t which we will save the calibration
 *           value to
 *   @param  tvoc_base
 *           A pointer to a uint16_t which we will save the calibration value to
 */
void sgp30_get_IAQ_baseline(sgp30_t *sensor, uint16_t *eco2_base, uint16_t *tvoc_base);

/**
 *  @brief  Assign baseline calibration values for both CO2 and TVOC IAQ
 *          calculations.
 *  @param  eco2_base
 *          A uint16_t which we will save the calibration value from
 *  @param  tvoc_base
 *          A uint16_t which we will save the calibration value from
 */
void sgp30_set_IAQ_baseline(sgp30_t *sensor, uint16_t eco2_base, uint16_t tvoc_base);

/**
 *  @brief  Set the absolute humidity value [mg/m^3] for compensation to
 *          increase precision of TVOC and eCO2.
 *  @param  absolute_humidity
 *          A uint32_t [mg/m^3] which we will be used for compensation.
 *          If the absolute humidity is set to zero, humidity compensation
 *          will be disabled.
 */
void sgp30_set_humidity(sgp30_t *sensor, uint32_t absolute_humidity);



/*********************************************************************************************
 * Measurement routine: START condition, the I2C WRITE header (7-bit I2C device address plus 0
 * as the write bit) and a 16-bit measurement command.
 *
 * All commands are listed in TABLE 10 on the datasheet. 
 * 
 * 
 * After the sensor has completed the measurement, the master can read the measurement results by sending a START
 * condition followed by an I2C READ header. The sensor will respond with the data.
 * 
 *! Each byte must be acknowledged by the microcontroller with an ACK condition for the sensor to continue sending data.
 * If the sensor does not receive an ACK from the master after any byte of data, it will not continue sending data.
 **********************************************************************************************/

/**
 *  @brief Executes commands based on SGP30 Command Table 
 * 
 *  @see https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Datasheets/Sensirion_Gas_Sensors_SGP30_Datasheet.pdf
 *       Table #10
 */

bool sgp30_execute_command(uint8_t command[], uint8_t command_len, uint16_t delay,
                           uint16_t *read_data, uint8_t read_len);

/**
 * @brief Data and commands are protected with a CRC checksum to increase the communication reliability.
 * The 16-bit commands that are sent to the sensor already include a 3-bit CRC checksum.
 * Data sent from and received by the sensor is always succeeded by an 8-bit CRC.
 *! In write direction it is mandatory to transmit the checksum, since the SGP30 only accepts data if it is followed by the correct
 *! checksum. 
 *
 ** In read direction it is up to the master to decide if it wants to read and process the checksum
 */
uint8_t sgp30_calculate_CRC(uint8_t *data, uint8_t len);


#endif  // __SGP30_H__