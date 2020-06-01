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

void Pd_Control_Law(Reference_data *reference)
{
//	Atualizar Valores de Referencia
//	rho.Th_ref = reference->rho;
	theta1.Th_ref = reference->theta1;
//	theta2.Th_ref = reference->theta2;
//	theta3.Th_ref = reference->theta3;

// Calcula Refer�ncia com o atraso
//	rho.dTh_ref = (rho.Th_ref - rho.Th_ref_anterior);
	theta1.dTh_ref = (theta1.Th_ref - theta1.Th_ref_anterior);
//	theta2.dTh_ref = (theta2.Th_ref - theta2.Th_ref_anterior);
//	theta3.dTh_ref = (theta3.Th_ref - theta3.Th_ref_anterior);

// Guarda os Valores de Refer�ncia
//	rho.Th_ref_anterior = rho.Th_ref;
	theta1.Th_ref_anterior = theta1.Th_ref;
//	theta2.Th_ref_anterior = theta2.Th_ref;
//	theta3.Th_ref_anterior = theta3.Th_ref;
// Calculo dos Diferenciais Th
//	rho.dTh = (rho.Th - rho.Th_anterior);
	theta1.dTh = (theta1.Th - theta1.Th_anterior);
//	theta2.dTh = (theta2.Th - theta2.Th_anterior);
//	theta3.dTh = (theta3.Th - theta3.Th_anterior);

//	Guarada os valores dos �ngulos
//	rho.Th_anterior = rho.Th;
	theta1.Th_anterior = theta1.Th;
//	theta2.Th_anterior = theta2.Th;
//	theta3.Th_anterior = theta3.Th;

// C�lculo do Sinal de Controle u[k]
//	Control_Signal (&rho);
	Control_Signal(&theta1);
//	Control_Signal (&theta2);
//	Control_Signal (&theta3);

}

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

    DR_angles_update(0); /* Angle Theta1 Update*/
	DR_angles_update(1);/* 'angulos_atualizados' update, Calculate for Kalman Filter*/
	

	 if(status_flag_print == 10){
	 printf("\n\rangle_updated[1]: %f", angulos_atualizados[1]);
	 printf("\n\r tempo_processamento = %0.4fms",tempo_processamento);
	 status_flag_print = 0;
	 }else{
	 status_flag_print ++;}
	 if(status_flag_diagnostic == 500){		
		 uint_fast8_t k = 0;
		for (k = 0 ;k < 5 ; k++ ){
		diagnostic_erro[k] = 0;
		}
		status_flag_diagnostic = 0;
	 }else{
		 status_flag_diagnostic ++;
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
	
	int16_t status_init;
	DR_mpu6050_ligar(1000);
	status_init = DR_mpu6050_init(&sensor_theta1);
	if (status_init != 1)
		printf("\n\rIncorrect startup: %d", status_init);

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
			{
				int status_angulo;
				status_angulo = DR_angles_update(0); /* Angle Theta1 Update*/
				DR_tick_start();
				status_angulo = DR_angles_update(1);/* 'angulos_atualizados' update, Calculate for Kalman Filter*/
				DR_tick_stop(false);
				printf("\n\rUpdate_return[1]: %d", status_angulo);
				printf("\n\rangle_updated[1]: %f", angulos_atualizados[1]);
			}
			if (contador == 2)
			{	/*Atualizar Sensor Theta 1*/
				int status_atualizar;
				DR_tick_start();
				status_atualizar = DR_mpu6050_atualizar(&sensor_theta1);
				DR_tick_stop(false);
				printf("\n\rRetorno de atualiza��o %d", status_atualizar);
				printf("\n\rSensor_theta1[ax]:%d\n\rSensor_theta1[ay]:%d\n\rSensor_theta1[az]:%d",
						sensor_theta1.ax, sensor_theta1.ay, sensor_theta1.az);
				printf("\n\rSensor_theta1[gx]:%d\n\rSensor_theta1[gy]:%d\n\rSensor_theta1[gz]:%d",
						sensor_theta1.gx, sensor_theta1.gy, sensor_theta1.gz);
				printf("\n\rSensor_theta1[temp]:%d", sensor_theta1.temp);
			}
			if (contador == 3)
			{	/*Atualizar Sensor Rho*/
				int status_atualizar;
				DR_tick_start();
				status_atualizar = DR_mpu6050_atualizar(&sensor_rho);
				DR_tick_stop(false);
				printf("\n\rRetorno de atualiza��o %d", status_atualizar);
				printf("\n\rSensor_rho[ax]:%d\n\rSensor_rho[ay]:%d\n\rSensor_rho[az]:%d",
						sensor_rho.ax, sensor_rho.ay, sensor_rho.az);
				printf("\n\rSensor_rho[gx]:%d\n\rSensor_rho[gy]:%d\n\rSensor_rho[gz]:%d",
						sensor_rho.gx, sensor_rho.gy, sensor_rho.gz);
				printf("\n\rSensor_rho[temp]:%d", sensor_rho.temp);
			}
			if (contador == 4)
			{ /* Primeiro Teste Ponto Flutuante*/

				FPU_enableModule();
//				FPU_enableLazyStacking();
				volatile float fCalculate;
				float ii;
				DR_tick_start();
				for (ii = 0.0f; ii < 20.0f; ii++)
				{
					fCalculate = (sinf(50.5f) * (12.2f / 50.1f) * 10.22f / 3.0f)
							* ii;
				}
				DR_tick_stop(false);
//				FPU_disableStacking();
//				FPU_disableModule();

			}
			if (contador == 5)
			{/* Segundo Teste Ponto Flutuante*/
				volatile double fCalculate;
				double ii;
				DR_tick_start();
				for (ii = 0; ii < 20; ii++)
				{
					fCalculate = (sin(50.5) * (12.2 / 50.1) * 10.22 / 3.0) * ii;
				}
				DR_tick_stop(false);
			}
			if (contador == 6)
			{/* Teste Cálculo das Funções */
				DR_tick_start();
				References_Circle(&Circle); // 3.29ms // 0.0211ms
				DR_tick_stop(false);
				DR_tick_start();
				Inverse_Kinematic(&Circle); // 5.92 - 6.20ms //0.1807ms
				DR_tick_stop(false);
				DR_tick_start();
				Pd_Control_Law(&Circle); // 0.0183ms
				DR_tick_stop(false);
			}
			if (contador == 7)
			{	/* Diagnostic Satus */
				Interrupt_disableInterrupt(INT_T32_INT1);			
				printf("Diagnostic Erro[0]: %d",diagnostic_erro[0]);
				printf("Diagnostic Erro[1]: %d",diagnostic_erro[1]);
				printf("Diagnostic Erro[2]: %d",diagnostic_erro[2]);
				printf("Diagnostic Erro[3]: %d",diagnostic_erro[3]);
				printf("Diagnostic Erro[4]: %d",diagnostic_erro[4]);
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
