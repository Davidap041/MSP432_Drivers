/* DriverLib Includes */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "Drivers_Control.h"
#include "mpu6050.h"
#include "Kalman.h"
#include "Control_Law.h"
#include "function.h"

uint8_t RXData[2];
uint8_t contador = 1;
// Vari�veis relacionadas ao filtro de kalman
// Instancia dos Filtros
Kalman_data kalman_0 = { .Q_angle = 0.1f, .Q_bias = 0.01f, .R_measure = 0.03f,
							.angle = 0, .bias = 0 };
Kalman_data kalman_1 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
									0.03f,
							.angle = 0.0f, .bias = 0.0f };
Kalman_data kalman_2 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
									0.03f,
							.angle = 0.0f, .bias = 0.0f };
Kalman_data kalman_3 = { .Q_angle = 0.0001f, .Q_bias = 0.01f,
							.R_measure = 0.03f, .angle = 0.0f, .bias = 0.0f };

// Vari�veis relacionadas aos �ngulos dos elos e controlador PD
Angle theta1 = { .identification = 1.0f, .kp_pd = 2.7f, .kv_pd = 0.3f, .m =
							0.2390f,
					.b = 0.98597f, .k = 0.001232f, .upper_limit = 1.972f,
					.lower_limit = 0.562f, .means = 1.3065f, .Th_ref = 0.0f };

// Defini��o do endere�amento dos 4 sensores
dr_mpu_data_t sensor_rho = { .identification = 0, .address = 0x68, .I2C = 0, };
dr_mpu_data_t sensor_theta1 = { .identification = 1, .address = 0x69, .I2C = 0, };
dr_mpu_data_t sensor_theta2 = { .identification = 2, .address = 0x68, .I2C = 2, };
dr_mpu_data_t sensor_theta3 = { .identification = 3, .address = 0x69, .I2C = 2, };
// Vari�veis para instancias as refer�ncias
Reference_data Circle = { .ref_time = pi_2 }, Triangle, Straight, Saw;

float angulos_atualizados[4];

uint_fast8_t status_flag_uart = 0;
uint_fast8_t status_flag_angle_update = 0;
uint_fast8_t status_flag_print = 0;
uint_fast16_t status_flag_diagnostic = 0;
double tempo_processamento = 0;
/* For Debug graph*/
uint8_t RXData[2];
uint8_t leitura[14];


int DR_angles_update(uint16_t n_sensor)
{
	if (n_sensor == 0)
	{
		int status_atualizar = DR_mpu6050_atualizar(&sensor_theta1);
	}
	if (n_sensor == 1)
	{
		/* Atualizar leitura do �ngulo theta 1*/
		float accX = sensor_theta1.ax * g * ACC_RESOLUTION;
		float accZ = sensor_theta1.az * g * ACC_RESOLUTION;
		float gyroY = sensor_theta1.gy * 1.3323e-04f;
		
		sensor_theta1.ang_gyro = gyroY;
		
		float pitch = atan2f(-accX, accZ);
		sensor_theta1.ang_pitch = pitch;

		float Kal_Angle = -getAngle(&kalman_1, pitch, gyroY, Ts);
		angulos_atualizados[1] = Kal_Angle;

		theta1.Th = angulos_atualizados[1];

		return 1;
	}
	if (n_sensor == 2)
	{
		return -3;
	}
	else
	{
		return 0;
	}
}

void interrupt_angles(){
    // Desativar a flag
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
    tempo_processamento = DR_tick_stop(false);
    DR_tick_start();
    // Leitura do Acelerêometro e Giroscopio
	DR_i2c_readraw(0,0x68,0x3B,14,leitura);
	sensor_rho.ax =		(leitura[0] << 8) | (leitura[1]);
	sensor_rho.ay = 	(leitura[2] << 8) | (leitura[3]);
	sensor_rho.az = 	(leitura[4] << 8) | (leitura[5]);
	sensor_rho.temp =  	(leitura[6] << 8) | (leitura[7]);
	sensor_rho.gx = 	(leitura[8] << 8) | (leitura[9]);
	sensor_rho.gy =		(leitura[10] << 8) | (leitura[11]);
	sensor_rho.gz = 	(leitura[12] << 8) | (leitura[13]);
	// Leitura do Magnetometro
	DR_i2c_readraw(0,0x0C,0x03,7,leitura);
	sensor_theta1.ax =		(leitura[0] << 8) | (leitura[1]);
	sensor_theta1.ay = 		(leitura[2] << 8) | (leitura[3]);
	sensor_theta1.az = 		(leitura[4] << 8) | (leitura[5]);
	sensor_theta1.temp =  	(leitura[6] << 8) | (leitura[7]);
	sensor_theta1.gx = 		(leitura[8] << 8) | (leitura[9]);
	sensor_theta1.gy =		(leitura[10] << 8) | (leitura[11]);
	sensor_theta1.gz = 		(leitura[12] << 8) | (leitura[13]);

	 if(status_flag_print == 10){
	 status_flag_print = 0;
	 }else{
	 status_flag_print ++;}
	 if(status_flag_diagnostic == 500){
	 }
}

int main(void)
{	WDT_A_holdTimer();

	/* Pin Config */
	DR_leds_sw_pin();
	DR_uart_pin();
	DR_i2c_pin();

	/* Peripherals Config */
	DR_uart_config(true);
	DR_i2c_config(0);
	DR_t32_config_Hz(0,100);
	
	/* Inicializar Programas*/
	DR_leds_init();
	DR_uart_init();
	DR_i2c_init(0);
	DR_t32_init(0);
	
	DR_mpu6050_ligar(1000);
	// status_erro = DR_mpu6050_read(sensor_rho.I2C, sensor_rho.address, 0x75, &data) - 10;
	// // while (data != 0x68);
	
	// Set accelerometers low pass filter at 5Hz
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address, 29, 0x06);
	// Set gyroscope low pass filter at 5Hz
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address, 26, 0x06);
	
	// Configure gyroscope range
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address,27, 0x10);
	// Configure accelerometers range
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address,28, 0x08);
	// Set by pass mode for the magnetometers
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address,0x37, 0x02);
	// Request continuous magnetometer measurements in 16 bits
	DR_i2c_write(sensor_rho.I2C, 0x0C,0x0A, 0x16);
	
	/* Interrupt Config */
	DR_uart_interrupt_receive();
	DR_interrupt_on();
	 DR_t32_interrupt_init(0,interrupt_angles);
	
	while (1)
	{

		if (status_flag_uart)
		{
			contador = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\r--Ledupdate(%d)",contador);
			if (contador == 1)
			{	/*Leitura do endereço*/
				int j;
				for (j = 0; j < 2; j++)
				printf("\n\rRxData[%d]:%x", j, RXData[j]);
			}
			if (contador == 2)
			{	/*Leitura acc e gyroscope*/
				int status_erro;
				status_erro = DR_i2c_readraw(0,0x68,0x3B,14,leitura);
				printf("\n\rRetorno do Status erro: %d", status_erro);
				int j;
				for (j = 0; j < 14; j++)
				printf("\n\rleitura[%d]:%x", j, leitura[j]);
				}
			if (contador == 3)
			{	/* Leitura Magnetometro endereço*/

			    DR_i2c_read(0,0x0C,0X02,RXData);
				int j;
				for (j = 0; j < 2; j++)
				printf("\n\rRxData[%d]:%x", j, RXData[j]);
			}		
			if (contador == 4)
			{	/* Leitura Magnetometro data*/
			int status_erro;
				status_erro = DR_i2c_readraw(0,0x0C,0x03,7,leitura);
				printf("\n\rRetorno do Status erro: %d", status_erro);
				int j;
				for (j = 0; j < 14; j++)
				printf("\n\rleitura[%d]:%x", j, leitura[j]);
				}
				
			}
			
			status_flag_uart = 0;
		}

}

void EUSCIA0_IRQHandler(void)
{
	UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
	status_flag_uart = 1;
}
