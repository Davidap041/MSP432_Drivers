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

Ctrl_Law_angle_data theta1 = { .identification = 1.0f, .kp_pd = 2.7f, .kv_pd = 0.3f, .m =
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
float magCalibration[3];
float magValue[3];
float magbias[3];
#define mRes 10.*4912./32760.0

float Magnetometer_offset[3];


int DR_angles_update(uint16_t n_sensor)
{
	if (n_sensor == 0)
	{
		int status_atualizar = DR_mpu6050_atualizar(&sensor_theta1);
	}
	if (n_sensor == 1)
	{
		/* Atualizar leitura do �ngulo theta 1*/
		float accX = magValue[0]; // Magnetometer X-axis // Testar multiplicar por 0.1
		float accY = magValue[1]; // Magnetometer Y-axis // Testar multiplicar por 0.1
		
		float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
		
		sensor_theta1.ang_gyro = gyroZ; // store Z euler angle gyroscope
		
		float pitch = atan2f(-accY, accX); // calculate angle Z from Magnetometer
		sensor_theta1.ang_pitch = pitch; // Store 
		// obs test the movement in a 180 degree in relation the angle of calibration
		float Kal_Angle = getAngle(&kalman_1, pitch, gyroZ, Ts);
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
	sensor_rho.ax =		(leitura[0] << 8) | (leitura[1]); // AccXH XL
	sensor_rho.ay = 	(leitura[2] << 8) | (leitura[3]); 
	sensor_rho.az = 	(leitura[4] << 8) | (leitura[5]);
	sensor_rho.temp =  	(leitura[6] << 8) | (leitura[7]);
	sensor_rho.gx = 	(leitura[8] << 8) | (leitura[9]);
	sensor_rho.gy =		(leitura[10] << 8) | (leitura[11]);
	sensor_rho.gz = 	(leitura[12] << 8) | (leitura[13]);
	// Leitura do Magnetometro
	DR_i2c_readraw(0,0x0C,0x03,7,leitura);
	sensor_theta1.ax =		(leitura[1] << 8) | (leitura[0]); // HXL HXH
	sensor_theta1.ay = 		(leitura[3] << 8) | (leitura[2]); 
	sensor_theta1.az = 		(leitura[4] << 8) | (leitura[4]);
	sensor_theta1.identification = leitura[7];	// Dado pronto 
	// // Calculo do magnetômetro
	// magbias[0] = +470.f;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    // magbias[1] = +120.f;  // User environmental x-axis correction in milliGauss
    // magbias[2] = +125.f;  // User environmental x-axis correction in milliGauss

	// magValue[0] = (float)sensor_theta1.ax*mRes*magCalibration[0] - magbias[0];
	// magValue[1] = (float)sensor_theta1.ay*mRes*magCalibration[1] - magbias[1];
	// magValue[2] = (float)sensor_theta1.az*mRes*magCalibration[2] - magbias[2];

	 magValue[0] = (float)sensor_theta1.ax - Magnetometer_offset[0];
	 magValue[1] = (float)sensor_theta1.ay - Magnetometer_offset[1];
	 magValue[2] = (float)sensor_theta1.az - Magnetometer_offset[2];
	// Calculate Kalman Filter Angle Rho
	DR_angles_update(1);

	 if(status_flag_print == 10){
	 status_flag_print = 0;
	 }else{
	 status_flag_print ++;}
	 if(status_flag_diagnostic == 500){
	 }
}
void Calibrar_Magnetometro(float *destination){
    int i = 500;
    int Max_Value[3];
    int Min_Value[3];

    DR_i2c_readraw(0,0x0C,0x03,7,leitura);
    sensor_theta1.ax =      (leitura[1] << 8) | (leitura[0]); // HXL HXH
    sensor_theta1.ay =      (leitura[3] << 8) | (leitura[2]);
    sensor_theta1.az =      (leitura[4] << 8) | (leitura[4]);

    Max_Value[0] = sensor_theta1.ax;
	Min_Value[0] = sensor_theta1.ax;
	Max_Value[1] = sensor_theta1.ay;
	Min_Value[1] = sensor_theta1.ay;
	Max_Value[2] = sensor_theta1.az;
	Min_Value[2] = sensor_theta1.az;

    while(i>0){
		DR_i2c_readraw(0,0x0C,0x03,7,leitura);
		sensor_theta1.ax =      (leitura[1] << 8) | (leitura[0]); // HXL HXH
		sensor_theta1.ay =      (leitura[3] << 8) | (leitura[2]);
		sensor_theta1.az =      (leitura[4] << 8) | (leitura[4]);
			// Maior e menor valor no eixo X
			if(sensor_theta1.ax > Max_Value[0]){
				Max_Value[0] = sensor_theta1.ax;
			}
			if(sensor_theta1.ax < Min_Value[0]){
				Min_Value[0] = sensor_theta1.ax;
			}
			// Maior e menor valor no eixo Y
			if(sensor_theta1.ay > Max_Value[1]){
				Max_Value[1] = sensor_theta1.ay;
			}
			if(sensor_theta1.ay < Min_Value[1]){
				Min_Value[1] = sensor_theta1.ay;
			}
			// Maior e menor valor no eixo Z
			if(sensor_theta1.az > Max_Value[2]){
				Max_Value[2] = sensor_theta1.az;
			}
			if(sensor_theta1.az < Min_Value[2]){
				Min_Value[2] = sensor_theta1.az;
			}
		DR_delay_k(0.01);
		i--;
    }
	destination[0] = (Max_Value[0]+Min_Value[0])/2;	// Offset eixo x
	destination[1] = (Max_Value[1]+Min_Value[1])/2; // Offset eixo y
	destination[2] = (Max_Value[2]+Min_Value[2])/2; // Offset eixo z
	

}
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  DR_i2c_write(0,0x0C, 0x0A, 0x00); // Power down magnetometer  
  DR_delay_k(1);
  DR_i2c_write(0,0x0C, 0x0A, 0x0F); // Enter Fuse ROM access mode
  DR_delay_k(1);
  DR_i2c_readraw(0,0x0C,0X10,3,&rawData[0]); // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  DR_i2c_write(0,0x0C, 0x0A, 0x00); // Power down magnetometer  
  DR_delay_k(1);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  DR_i2c_write(0,0x0C, 0x0A, 1 << 4 | 0x02); // Set magnetometer data resolution and sample ODR
  DR_delay_k(1);
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
	
	DR_mpu6050_ligar(10); // I'm not using this now!
	// status_erro = DR_mpu6050_read(sensor_rho.I2C, sensor_rho.address, 0x75, &data) - 10;
	// // while (data != 0x68);
	
	// Set accelerometers low pass filter at 5Hz (ACCEL_CONFIG 2)
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address, 29, 0x06);
	// Set gyroscope low pass filter at 5Hz (CONFIG)
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address, 26, 0x06);
	
	// Configure gyroscope range(GYRO_FS_SEL)
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address,27, 0x10);
	// Configure accelerometers range (ACCEL_FS_SEL)
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address,28, 0x08);
	// Set by pass mode for the magnetometers
	DR_i2c_write(sensor_rho.I2C, sensor_rho.address,0x37, 0x02);
	// Request continuous magnetometer measurements in 16 bits !!!
	DR_i2c_write(sensor_rho.I2C, 0x0C,0x0A, 0x16);
	printf("\n\rSensores Inicializados");
	// Auto Calibração do Magnetômetro
	Calibrar_Magnetometro(Magnetometer_offset);
    printf("\n\rSensores Calibrados");
    printf("\n\rOffsetX : %.4f",Magnetometer_offset[0]);
    printf("\n\rOffsetY : %.4f",Magnetometer_offset[1]);
    printf("\n\rOffsetZ : %.4f",Magnetometer_offset[2]);
	
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
				DR_i2c_read(0,0x68,0x75,RXData);
				printf("\n\rMPU9250 i AM : 0x%x should be : 0x71",RXData[0]);
				
				DR_i2c_read(0,0x0C,0x00,RXData);
				printf("\n\rMPU9250 i AM : 0x%x should be: 0X48",RXData[0]);
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
			{	/* Leitura Magnetometro calibração*/
				float magCalibration[3];
				initAK8963(magCalibration);
				
				printf("\n\rMag:X-Axis sensitivity adjustment value %.5f",magCalibration[0]);
				printf("\n\rMag:Y-Axis sensitivity adjustment value %.5f",magCalibration[1]);
				printf("\n\rMag:Z-Axis sensitivity adjustment value %.5f",magCalibration[2]);

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
