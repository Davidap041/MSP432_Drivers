/* DriverLib Includes */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "Drivers_Control.h"
#include "mpu6050.h"

uint8_t RXData[2];
uint8_t contador = 1;


// Definição do endereçamento dos 4 sensores
mpu_data_t sensor_rho =    { .identification = 0, .address = 0x68, .I2C = 0, };
mpu_data_t sensor_theta1 = { .identification = 1, .address = 0x69, .I2C = 0, };
mpu_data_t sensor_theta2 = { .identification = 2, .address = 0x68, .I2C = 2, };
mpu_data_t sensor_theta3 = { .identification = 3, .address = 0x69, .I2C = 2, };
// tempo de leitura dos quatro sensores estimado em 2,1424ms(sem falhas) -
// Flags
uint32_t status_flag_uart = 0;

int main(void)
{

	WDT_A_holdTimer();
	/* Pin Config */
	dr_Leds_sw_init();
	dr_I2C0_pin_config();

	/* Peripherals Config */
	dr_Uart_init();
	dr_I2C_init(0);

	/* Interrupt Config */
	dr_Uart_interrupt_receive();
	dr_Interrupt_on();

	/* Inicializar Programas*/
	int16_t status_init;
	dr_MPU6050_ligar(500);
	status_init = dr_MPU6050_init(&sensor_rho);
	if(status_init != 1)
	printf("\n\rInicialização incorreta: %d",status_init);

	while (1)
	{
		dr_Leds_alterar(contador);
		dr_Delay_s(1);

		if (status_flag_uart)
		{
			contador = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\r-------Atualizando o valor para os Leds em: %d ---------",
					contador);
			if (contador == 2)
			{
				int status_atualizar;
				dr_Tick_start();
				status_atualizar = dr_MPU6050_atualizar(&sensor_theta1);
				dr_Tick_stop();
				printf("\n\rRetorno de atualização %d",status_atualizar);
				printf("\n\rSensor_theta1[ax]:%d\n\rSensor_theta1[ay]:%d\n\rSensor_theta1[az]:%d",
						sensor_theta1.ax, sensor_theta1.ay, sensor_theta1.az);
				printf("\n\rSensor_theta1[gx]:%d\n\rSensor_theta1[gy]:%d\n\rSensor_theta1[gz]:%d",
						sensor_theta1.gx, sensor_theta1.gy, sensor_theta1.gz);
				printf("\n\rSensor_theta1[temp]:%d", sensor_theta1.temp);
			}
			if (contador == 3)
			{
				int status_atualizar;
				dr_Tick_start();
				status_atualizar = dr_MPU6050_atualizar(&sensor_rho);
				dr_Tick_stop();
				printf("\n\rRetorno de atualização %d",status_atualizar);
				printf("\n\rSensor_rho[ax]:%d\n\rSensor_rho[ay]:%d\n\rSensor_rho[az]:%d",
						sensor_rho.ax, sensor_rho.ay, sensor_rho.az);
				printf("\n\rSensor_rho[gx]:%d\n\rSensor_rho[gy]:%d\n\rSensor_rho[gz]:%d",
					   sensor_rho.gx, sensor_rho.gy, sensor_rho.gz);
				printf("\n\rSensor_rho[temp]:%d", sensor_rho.temp);
			}

			status_flag_uart = 0;
		}
	}
}

void EUSCIA0_IRQHandler(void)
{
	UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
	status_flag_uart = 1;
}
