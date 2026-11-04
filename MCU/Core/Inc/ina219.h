/*
 * ina219.h
 *
 * Driver minimal INA219 (I2C) - shunt 0.1 Ohm
 * Tout est en entiers (pas de printf float necessaire)
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "main.h"
#include <stdint.h>
#include <stddef.h>

#define INA219_ADDR        (0x40 << 1)  // A0 = A1 = GND (adresse 7 bits 0x40 decalee pour la HAL)
#define INA219_SHUNT_MOHM  100          // Resistance de shunt en milliohms (0.1 Ohm)

typedef struct
{
    int32_t bus_mV;     // Tension bus (V+ a GND cote charge) en mV
    int32_t shunt_uV;   // Tension aux bornes du shunt en uV
    int32_t current_uA; // Courant en uA (signe : negatif = courant inverse)
    int32_t power_mW;   // Puissance en mW
    uint8_t overflow;   // Flag OVF du registre bus
} INA219_Data_t;

HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef INA219_Read(INA219_Data_t *data);
HAL_StatusTypeDef INA219_Recover(void);                 // Re-init I2C + INA219 apres une erreur
void INA219_Format(const INA219_Data_t *d, char *out, size_t len);

#endif /* INC_INA219_H_ */
