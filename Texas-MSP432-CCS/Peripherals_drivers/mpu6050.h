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
	/* Sensor Address*/
	uint8_t address;
	/* Sensor I2C in use */
	uint8_t I2C;
	/* Acceleration values from Accelerometer*/
	int16_t ax;
	int16_t ay;
	int16_t az;
	/* Temperature value */
	int16_t temp;
	/* Gyre values from gyroscope in Euler Angle */
	int16_t gx;
	int16_t gy;
	int16_t gz;
	/* Magnet Field from Magnetometer */
	int16_t mx;
	int16_t my;
	int16_t mz;

	/* Magnetometer Calibration Variables*/
	float mag_offset_x;
	float mag_offset_y;
	float mag_offset_z;

	/* Giroscope Calibration Variables*/
	float gyro_offset_x;
	float gyro_offset_y;
	float gyro_offset_z;

	/* Input variables for the Kalman Filter*/
	float ang_pitch;
	float ang_gyro;

	/* Final Angle Update by kalman Filter*/
	float ang_updated;
	float ang_Kalman;
	float ang_Luenberger;

} dr_mpu_data_t;

int16_t erro_watch[5];
uint32_t diagnostic_erro[5];
/*Atualizar valores de ax,ay,az,temp,gx,gy,gz*/
extern int DR_mpu6050_atualizar(dr_mpu_data_t *sensor);
extern int DR_mpu6050_init(dr_mpu_data_t *sensor);
extern void DR_mpu6050_ligar(uint16_t tempo_ms);
extern int DR_mpu6050_readraw(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
								uint8_t byteCount, uint8_t *data);
extern int DR_mpu6050_read(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
							uint8_t *data);
extern void DR_Gyroscope_calibrate(dr_mpu_data_t *sensor);
extern void DR_magnetometer_calibrate(dr_mpu_data_t *sensor);
extern void initAK8963(float * destination);
extern int  DR_mpu9250_init(dr_mpu_data_t *sensor);
extern int DR_mpu9250_atualizar(dr_mpu_data_t *sensor);



//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_CONTROL_H_ */
