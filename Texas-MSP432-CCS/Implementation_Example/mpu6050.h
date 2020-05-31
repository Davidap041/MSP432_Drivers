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
// Includes necess�rios
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h> // usar essa defini��o para o CodeComposer
#include "Drivers_Control.h"

#define g 9.8f
#define ACC_RESOLUTION (1.0f/16384.0f)

typedef struct
{
	uint8_t identification;
	/* Endere�o do sensor*/
	uint8_t address;
	/* I2C conectado */
	uint8_t I2C;
	/* Valores de Acelera��o do aceler�metro*/
	int16_t ax;
	int16_t ay;
	int16_t az;
	/* Valor de Temperatura */
	int16_t temp;
	/* Valores de Acelera��o do Gisrosc�pip */
	int16_t gx;
	int16_t gy;
	int16_t gz;

} dr_mpu_data_t;

int16_t erro_watch[5];
uint32_t diagnostic_erro[5];
extern int DR_mpu6050_atualizar(dr_mpu_data_t *sensor);
extern int DR_mpu6050_init(dr_mpu_data_t *sensor);
extern void DR_mpu6050_ligar(uint16_t tempo_ms);
extern int DR_mpu6050_readraw(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
								uint8_t byteCount, uint8_t *data);
extern int DR_mpu6050_read(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
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
