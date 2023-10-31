

#include "spl06.h"
#include "../bus/i2c_soft.h"
#include "barometer.h"
#include "../schedule/ticks.h"
#include <math.h>
#include <stdio.h>

// SPL06, address 0x76

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06_coeffs_t;


spl06_coeffs_t spl06_cal;
// uncompensated pressure and temperature
static int32_t spl06_pressure_raw = 0;
static int32_t spl06_temperature_raw = 0;

static int8_t spl06_samples_to_cfg_reg_value(uint8_t sample_rate)
{
    switch(sample_rate)
    {
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        case 128: return 7;
        default: return -1; // invalid
    }
}

static int32_t spl06_raw_value_scale_factor(uint8_t oversampling_rate)
{
    switch(oversampling_rate)
    {
        case 1: return 524288;
        case 2: return 1572864;
        case 4: return 3670016;
        case 8: return 7864320;
        case 16: return 253952;
        case 32: return 516096;
        case 64: return 1040384;
        case 128: return 2088960;
        default: return -1; // invalid
    }
}

static bool spl06_start_temperature_measurement(void)
{
    return soft_i2c_write_reg(SPL06_I2C_ADDR, SPL06_MODE_AND_STATUS_REG, SPL06_MEAS_TEMPERATURE);
}

static bool spl06_read_temperature(void)
{
    uint8_t data[SPL06_TEMPERATURE_LEN];
    int32_t spl06_temperature;

    bool ack = !soft_i2c_buffer_read(SPL06_I2C_ADDR, SPL06_TEMPERATURE_START_REG, data, SPL06_TEMPERATURE_LEN);

    if (ack) {
        spl06_temperature = (int32_t)((data[0] & 0x80 ? 0xFF000000 : 0) | (((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
        spl06_temperature_raw = spl06_temperature;
    }

    return ack;
}

static bool spl06_start_pressure_measurement(void)
{
    return soft_i2c_write_reg(SPL06_I2C_ADDR, SPL06_MODE_AND_STATUS_REG, SPL06_MEAS_PRESSURE);
}

static bool spl06_read_pressure(void)
{
    uint8_t data[SPL06_PRESSURE_LEN];
    int32_t spl06_pressure;

    bool ack = !soft_i2c_buffer_read(SPL06_I2C_ADDR, SPL06_PRESSURE_START_REG, data, SPL06_PRESSURE_LEN);

    if (ack) {
        spl06_pressure = (int32_t)((data[0] & 0x80 ? 0xFF000000 : 0) | (((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
        spl06_pressure_raw = spl06_pressure;
    }

    return ack;
}

// Returns temperature in degrees centigrade
static float spl06_compensate_temperature(int32_t temperature_raw)
{
    const float t_raw_sc = (float)temperature_raw / spl06_raw_value_scale_factor(SPL06_TEMPERATURE_OVERSAMPLING);
    const float temp_comp = (float)spl06_cal.c0 / 2 + t_raw_sc * spl06_cal.c1;
    return temp_comp;
}

// Returns pressure in Pascal
static float spl06_compensate_pressure(int32_t pressure_raw, int32_t temperature_raw)
{
    const float p_raw_sc = (float)pressure_raw / spl06_raw_value_scale_factor(SPL06_PRESSURE_OVERSAMPLING);
    const float t_raw_sc = (float)temperature_raw / spl06_raw_value_scale_factor(SPL06_TEMPERATURE_OVERSAMPLING);

    const float pressure_cal = (float)spl06_cal.c00 + p_raw_sc * ((float)spl06_cal.c10 + p_raw_sc * ((float)spl06_cal.c20 + p_raw_sc * spl06_cal.c30));
    const float p_temp_comp = t_raw_sc * ((float)spl06_cal.c01 + p_raw_sc * ((float)spl06_cal.c11 + p_raw_sc * spl06_cal.c21));

    return pressure_cal + p_temp_comp;
}

bool spl06_calculate(int32_t * pressure, int32_t * temperature)
{
    if (pressure) {
        *pressure = lrintf(spl06_compensate_pressure(spl06_pressure_raw, spl06_temperature_raw));
    }

    if (temperature) {
        *temperature = lrintf(spl06_compensate_temperature(spl06_temperature_raw) * 100);
    }
    
    return true;
}

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(void)
{
    uint8_t chipId;
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        delay_ms(10);
        chipId = soft_i2c_read_reg(SPL06_I2C_ADDR, SPL06_CHIP_ID_REG);
        if (chipId == SPL06_DEFAULT_CHIP_ID) {
            return true;
        }
    };

    return false;
}

static bool read_calibration_coefficients(void) {
    uint8_t sstatus = soft_i2c_read_reg(SPL06_I2C_ADDR, SPL06_MODE_AND_STATUS_REG);
    if (!(sstatus & SPL06_MEAS_CFG_COEFFS_RDY))
        return false;   // error reading status or coefficients not ready

    uint8_t caldata[SPL06_CALIB_COEFFS_LEN];

    if (soft_i2c_buffer_read(SPL06_I2C_ADDR, SPL06_CALIB_COEFFS_START, (uint8_t *)&caldata, SPL06_CALIB_COEFFS_LEN)) {
        return false;
    }

    spl06_cal.c0 = (caldata[0] & 0x80 ? 0xF000 : 0) | ((uint16_t)caldata[0] << 4) | (((uint16_t)caldata[1] & 0xF0) >> 4);
    spl06_cal.c1 = ((caldata[1] & 0x8 ? 0xF000 : 0) | ((uint16_t)caldata[1] & 0x0F) << 8) | (uint16_t)caldata[2];
    spl06_cal.c00 = (caldata[3] & 0x80 ? 0xFFF00000 : 0) | ((uint32_t)caldata[3] << 12) | ((uint32_t)caldata[4] << 4) | (((uint32_t)caldata[5] & 0xF0) >> 4);
    spl06_cal.c10 = (caldata[5] & 0x8 ? 0xFFF00000 : 0) | (((uint32_t)caldata[5] & 0x0F) << 16) | ((uint32_t)caldata[6] << 8) | (uint32_t)caldata[7];
    spl06_cal.c01 = ((uint16_t)caldata[8] << 8) | ((uint16_t)caldata[9]);
    spl06_cal.c11 = ((uint16_t)caldata[10] << 8) | (uint16_t)caldata[11];
    spl06_cal.c20 = ((uint16_t)caldata[12] << 8) | (uint16_t)caldata[13];
    spl06_cal.c21 = ((uint16_t)caldata[14] << 8) | (uint16_t)caldata[15];
    spl06_cal.c30 = ((uint16_t)caldata[16] << 8) | (uint16_t)caldata[17];

    return true;
}

static bool spl06_configure_measurements(void)
{
    uint8_t reg_value;

    reg_value = SPL06_TEMP_USE_EXT_SENSOR | spl06_samples_to_cfg_reg_value(SPL06_TEMPERATURE_OVERSAMPLING);
    if (soft_i2c_write_reg(SPL06_I2C_ADDR, SPL06_TEMPERATURE_CFG_REG, reg_value)) {
        return false;
    }

    reg_value = spl06_samples_to_cfg_reg_value(SPL06_PRESSURE_OVERSAMPLING);
    if (soft_i2c_write_reg(SPL06_I2C_ADDR, SPL06_PRESSURE_CFG_REG, reg_value)) {
        return false;
    }

    reg_value = 0;
    if (SPL06_TEMPERATURE_OVERSAMPLING > 8) {
        reg_value |= SPL06_TEMPERATURE_RESULT_BIT_SHIFT;
    }
    if (SPL06_PRESSURE_OVERSAMPLING > 8) {
        reg_value |= SPL06_PRESSURE_RESULT_BIT_SHIFT;
    }
    if (soft_i2c_write_reg(SPL06_I2C_ADDR, SPL06_INT_AND_FIFO_CFG_REG, reg_value)) {
        return false;
    }

    return true;
}

bool spl06Detect(baroDev_t *baro)
{
    if (!(deviceDetect() && read_calibration_coefficients() && spl06_configure_measurements())) {
        printf("baro detect error!\r\n");        
        return false;
    }

    baro->ut_delay = SPL06_MEASUREMENT_TIME(SPL06_TEMPERATURE_OVERSAMPLING);
    baro->get_ut = spl06_read_temperature;
    baro->start_ut = spl06_start_temperature_measurement;

    baro->up_delay = SPL06_MEASUREMENT_TIME(SPL06_PRESSURE_OVERSAMPLING);
    baro->start_up = spl06_start_pressure_measurement;
    baro->get_up = spl06_read_pressure;

    baro->calculate = spl06_calculate;

//    //--------------------------------------------------------
//    printf("spl06_cal.c0: %d\r\n", spl06_cal.c0);
//    printf("spl06_cal.c1: %d\r\n", spl06_cal.c1);
//    printf("spl06_cal.c00: %d\r\n", spl06_cal.c00);
//    printf("spl06_cal.c10: %d\r\n", spl06_cal.c10);
//    printf("spl06_cal.c01: %d\r\n", spl06_cal.c01);
//    printf("spl06_cal.c11: %d\r\n", spl06_cal.c11);
//    printf("spl06_cal.c20: %d\r\n", spl06_cal.c20);
//    printf("spl06_cal.c21: %d\r\n", spl06_cal.c21);
//    printf("spl06_cal.c30: %d\r\n", spl06_cal.c30);
    
    return true;
}


