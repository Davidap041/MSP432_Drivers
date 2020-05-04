/*
 * Drivers_Control.h
 *
 *  Created on: 21 de mar de 2020
 *      Author: davia
 * Biblioteca para auxiliar o uso da SDK com alguns exemplos
 */
#ifndef MPU6050_H_
#define MPU6050_H_

//*****************************************************************************
//
//! \addtogroup gpio_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************************************************************************************************
// Includes necessários
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h> // usar essa definição para o CodeComposer
#include "Drivers_Control.h"

typedef struct
{
	uint8_t identification;
	/* Endereço do sensor*/
	uint8_t address;
	/* I2C conectado */
	uint8_t I2C;
	/* Valores de Aceleração do acelerômetro*/
	int16_t ax;
	int16_t ay;
	int16_t az;
	/* Valor de Temperatura */
	int16_t temp;
	/* Valores de Aceleração do Gisroscópip */
	int16_t gx;
	int16_t gy;
	int16_t gz;

} mpu_data_t;
int16_t erro_watch[5];
extern int dr_MPU6050_atualizar(mpu_data_t *sensor);
extern int dr_MPU6050_init(mpu_data_t *sensor);
extern void dr_MPU6050_ligar(uint16_t tempo_ms);
extern int dr_MPU6050_ReadRaw(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
								uint8_t byteCount, uint8_t *data);
extern int dr_MPU6050_Read(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
							uint8_t *data);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_CONTROL_H_ */
