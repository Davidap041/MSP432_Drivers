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
// Variáveis relacionadas ao filtro de kalman
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

// Variáveis relacionadas aos ângulos dos elos e controlador PD
Angle theta1 = { .identification = 1.0f, .kp_pd = 2.7f, .kv_pd = 0.3f, .m =
							0.2390f,
					.b = 0.98597f, .k = 0.001232f, .upper_limit = 1.972f,
					.lower_limit = 0.562f, .means = 1.3065f, .Th_ref = 0.0f };

// Definição do endereçamento dos 4 sensores
mpu_data_t sensor_rho = { .identification = 0, .address = 0x68, .I2C = 0, };
mpu_data_t sensor_theta1 = { .identification = 1, .address = 0x69, .I2C = 0, };
mpu_data_t sensor_theta2 = { .identification = 2, .address = 0x68, .I2C = 2, };
mpu_data_t sensor_theta3 = { .identification = 3, .address = 0x69, .I2C = 2, };
// Variáveis para instancias as referências
Reference_data Circle = { .ref_time = pi_2 }, Triangle, Straight, Saw;

float angulos_atualizados[4];

uint32_t status_flag_uart = 0;
void Pd_Control_Law(Reference_data *reference)
{
//	Atualizar Valores de Referencia
//	rho.Th_ref = reference->rho;
	theta1.Th_ref = reference->theta1;
//	theta2.Th_ref = reference->theta2;
//	theta3.Th_ref = reference->theta3;

// Calcula Referência com o atraso
//	rho.dTh_ref = (rho.Th_ref - rho.Th_ref_anterior);
	theta1.dTh_ref = (theta1.Th_ref - theta1.Th_ref_anterior);
//	theta2.dTh_ref = (theta2.Th_ref - theta2.Th_ref_anterior);
//	theta3.dTh_ref = (theta3.Th_ref - theta3.Th_ref_anterior);

// Guarda os Valores de Referência
//	rho.Th_ref_anterior = rho.Th_ref;
	theta1.Th_ref_anterior = theta1.Th_ref;
//	theta2.Th_ref_anterior = theta2.Th_ref;
//	theta3.Th_ref_anterior = theta3.Th_ref;
// Calculo dos Diferenciais Th
//	rho.dTh = (rho.Th - rho.Th_anterior);
	theta1.dTh = (theta1.Th - theta1.Th_anterior);
//	theta2.dTh = (theta2.Th - theta2.Th_anterior);
//	theta3.dTh = (theta3.Th - theta3.Th_anterior);

//	Guarada os valores dos Ângulos
//	rho.Th_anterior = rho.Th;
	theta1.Th_anterior = theta1.Th;
//	theta2.Th_anterior = theta2.Th;
//	theta3.Th_anterior = theta3.Th;

// Cálculo do Sinal de Controle u[k]
//	Control_Signal (&rho);
	Control_Signal(&theta1);
//	Control_Signal (&theta2);
//	Control_Signal (&theta3);

}

int dr_angulos_atualizar(uint16_t n_sensor)
{
	if (n_sensor == 0)
	{
		int status_atualizar = dr_MPU6050_atualizar(&sensor_theta1);
	}
	if (n_sensor == 1)
	{
		/* Atualizar leitura do ângulo theta 1*/
		float accX = sensor_theta1.ax * g * ACC_RESOLUTION;
		float accZ = sensor_theta1.az * g * ACC_RESOLUTION;
		float gyroY = sensor_theta1.gy * 1.3323e-04f;

		float pitch = atan2f(-accX, accZ);

		float Kal_Angle = -getAngle(&kalman_1, pitch, gyroY, Ts);
		angulos_atualizados[1] = Kal_Angle;

		float dif_ruido = (angulos_atualizados[1] - theta1.Th) * 1000.0f;
		theta1.dif_ruido = abs(dif_ruido);

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
	if (status_init != 1)
		printf("\n\rInicialização incorreta: %d", status_init);

	while (1)
	{
		dr_Leds_alterar(contador);
		dr_Delay_s(1);

		if (status_flag_uart)
		{
			contador = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\r-------Atualizando o valor para os Leds em: %d ---------",
					contador);
			if (contador == 1)
			{
				int status_angulo;
				status_angulo = dr_angulos_atualizar(0);
				dr_Tick_start();
				status_angulo = dr_angulos_atualizar(1);
				dr_Tick_stop();
				printf("\n\rRetorno de atualização[1] %d", status_angulo);
				printf("\n\rAngulo_atualizado[1] :%f", angulos_atualizados[1]);
				dr_Tick_start();
				status_angulo = dr_angulos_atualizar(2);
				dr_Tick_stop();
				printf("\n\rRetorno de atualização[2] %d", status_angulo);
				printf("\n\rAngulo_atualizado[2] :%f", angulos_atualizados[1]);
//				printf("\n\raccX:%f,accZ:%f,gyroY:%f\n\rpitch:%f,Kal_Angle:%f",
//						accX, accZ, gyroY, pitch, Kal_Angle);

			}
			if (contador == 2)
			{
				int status_atualizar;
				dr_Tick_start();
				status_atualizar = dr_MPU6050_atualizar(&sensor_theta1);
				dr_Tick_stop();
				printf("\n\rRetorno de atualização %d", status_atualizar);
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
				printf("\n\rRetorno de atualização %d", status_atualizar);
				printf("\n\rSensor_rho[ax]:%d\n\rSensor_rho[ay]:%d\n\rSensor_rho[az]:%d",
						sensor_rho.ax, sensor_rho.ay, sensor_rho.az);
				printf("\n\rSensor_rho[gx]:%d\n\rSensor_rho[gy]:%d\n\rSensor_rho[gz]:%d",
						sensor_rho.gx, sensor_rho.gy, sensor_rho.gz);
				printf("\n\rSensor_rho[temp]:%d", sensor_rho.temp);
			}
			if (contador == 4)
			{ /* Primeiro Teste*/

				FPU_enableModule();
//				FPU_enableLazyStacking();
				volatile float fCalculate;
				float ii;
				dr_Tick_start();
				for (ii = 0.0f; ii < 20.0f; ii++)
				{
					fCalculate = (sinf(50.5f) * (12.2f / 50.1f) * 10.22f / 3.0f)
							* ii;
				}
				dr_Tick_stop();
//				FPU_disableStacking();
//				FPU_disableModule();

			}
			if (contador == 5)
			{
				volatile double fCalculate;
				double ii;
				dr_Tick_start();
				for (ii = 0; ii < 20; ii++)
				{
					fCalculate = (sin(50.5) * (12.2 / 50.1) * 10.22 / 3.0) * ii;
				}
				dr_Tick_stop();
			}
			if (contador == 6)
			{
				dr_Tick_start();
				References_Circle(&Circle); // 3.29ms // 0.0211ms
				dr_Tick_stop();
				dr_Tick_start();
				Inverse_Kinematic(&Circle); // 5.92 - 6.20ms //0.1807ms
				dr_Tick_stop();
				dr_Tick_start();
				Pd_Control_Law(&Circle); // 0.0183ms
				dr_Tick_stop();
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
