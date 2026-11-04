/*
 * ina219.c
 *
 * Mesure du courant directement a partir du registre shunt (LSB = 10 uV),
 * donc pas besoin du registre de calibration.
 *   I = Vshunt / R  ->  avec R = 0.1 Ohm : 1 LSB = 100 uA
 * Plage : +/- 320 mV sur le shunt = +/- 3.2 A
 */

#include "ina219.h"
#include <stdio.h>

#define INA219_REG_CONFIG   0x00
#define INA219_REG_SHUNT    0x01
#define INA219_REG_BUS      0x02

#define INA219_I2C_TIMEOUT  20 // ms

/*
 * Config :
 *  BRNG = 1  (bus 32 V)             -> 0x2000
 *  PG   = 11 (+/-320 mV)            -> 0x1800
 *  BADC = 1011 (moyenne 16 echant.) -> 0x0580
 *  SADC = 1011 (moyenne 16 echant.) -> 0x0058
 *  MODE = 111 (shunt + bus continu) -> 0x0007
 *  Temps de conversion : ~8.5 ms par voie
 */
#define INA219_CONFIG_VALUE 0x3DDF

static I2C_HandleTypeDef *ina_i2c = NULL;

static HAL_StatusTypeDef ina_write16(uint8_t reg, uint16_t val)
{
    uint8_t b[2] = {(uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return HAL_I2C_Mem_Write(ina_i2c, INA219_ADDR, reg, I2C_MEMADD_SIZE_8BIT, b, 2, INA219_I2C_TIMEOUT);
}

static HAL_StatusTypeDef ina_read16(uint8_t reg, uint16_t *val)
{
    uint8_t b[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(ina_i2c, INA219_ADDR, reg, I2C_MEMADD_SIZE_8BIT, b, 2, INA219_I2C_TIMEOUT);
    if (st == HAL_OK)
    {
        *val = ((uint16_t)b[0] << 8) | b[1]; // L'INA219 envoie le MSB en premier
    }
    return st;
}

HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c)
{
    ina_i2c = hi2c;

    if (HAL_I2C_IsDeviceReady(ina_i2c, INA219_ADDR, 3, INA219_I2C_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return ina_write16(INA219_REG_CONFIG, INA219_CONFIG_VALUE);
}

HAL_StatusTypeDef INA219_Recover(void)
{
    if (ina_i2c == NULL)
        return HAL_ERROR;

    HAL_I2C_DeInit(ina_i2c);
    if (HAL_I2C_Init(ina_i2c) != HAL_OK)
        return HAL_ERROR;

    return INA219_Init(ina_i2c);
}

HAL_StatusTypeDef INA219_Read(INA219_Data_t *d)
{
    uint16_t raw_shunt, raw_bus;
    HAL_StatusTypeDef st;

    st = ina_read16(INA219_REG_SHUNT, &raw_shunt);
    if (st != HAL_OK)
        return st;

    st = ina_read16(INA219_REG_BUS, &raw_bus);
    if (st != HAL_OK)
        return st;

    d->shunt_uV   = (int32_t)(int16_t)raw_shunt * 10;                 // LSB = 10 uV
    d->bus_mV     = (int32_t)(raw_bus >> 3) * 4;                      // LSB = 4 mV, bits [15:3]
    d->overflow   = raw_bus & 0x01;
    d->current_uA = (d->shunt_uV * 1000) / INA219_SHUNT_MOHM;         // uV / mOhm = mA -> *1000 = uA
    d->power_mW   = (int32_t)(((int64_t)d->bus_mV * d->current_uA) / 1000000LL);

    return HAL_OK;
}

void INA219_Format(const INA219_Data_t *d, char *out, size_t len)
{
    int32_t ma10 = d->current_uA / 100;           // courant en 0.1 mA
    const char *sign = (ma10 < 0) ? "-" : "";
    if (ma10 < 0)
        ma10 = -ma10;

    snprintf(out, len, "INA219: V=%ld.%03ld V I=%s%ld.%ld mA P=%ld mW%s\r\n",
             (long)(d->bus_mV / 1000), (long)(d->bus_mV % 1000),
             sign, (long)(ma10 / 10), (long)(ma10 % 10),
             (long)d->power_mW,
             d->overflow ? " OVF" : "");
}
