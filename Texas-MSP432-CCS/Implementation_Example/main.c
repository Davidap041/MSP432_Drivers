#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
/* DriverLib Includes */
#include "Drivers_Control.h"
#include "mpu6050.h"
#include "Kalman.h"
#include "Control_Law.h"
#include "function.h"

/* Instancia dos Objetos */
// PWM Configuration for Channel
// Drive PWM for link Rho in 50Hz
dr_pwm_parameters PWM_Rho = {
	.identification = 0,
	.timer = TIMER_A0_BASE,
	.fast_mode = true,
	.timer_Prescaler = 4,
	.true_Sawtooth_not_triangular = true,
	.period_count = 60000, //60360 = 50 Hz(fechado, osciloscópio)
	.pwm_channel = 1,
	.outputmode = TIMER_A_OUTPUTMODE_RESET_SET
};
// Drive PWM for link Theta1 in 50Hz
dr_pwm_parameters PWM_Theta1 = {
	.identification = 1,
	.timer = TIMER_A0_BASE,
	.fast_mode = true,
	.timer_Prescaler = 4,
	.true_Sawtooth_not_triangular = true,
	.period_count = 60000,	// 60000 = 50,33Hz (osciloscópio)
	.pwm_channel = 2,
	.outputmode = TIMER_A_OUTPUTMODE_RESET_SET
};
// Drive PWM for link Theta2 in 50Hz
dr_pwm_parameters PWM_Theta2 = {
	.identification = 2,
	.timer = TIMER_A0_BASE,
	.fast_mode = true,
	.timer_Prescaler = 4,
	.true_Sawtooth_not_triangular = true,
	.period_count = 60000,
	.pwm_channel = 3,
	.outputmode = TIMER_A_OUTPUTMODE_RESET_SET
};
// Drive PWM for link Theta3 in 50Hz
dr_pwm_parameters PWM_Theta3 = {
	.identification = 3,
	.timer = TIMER_A0_BASE,
	.fast_mode = true,
	.timer_Prescaler = 4,
	.true_Sawtooth_not_triangular = true,
	.period_count = 60000,
	.pwm_channel = 4,
	.outputmode = TIMER_A_OUTPUTMODE_RESET_SET
};
// Sensor Configuration and Address
// Sensor (accelerometer + Gyroscope) for measure angle Rho
dr_mpu_data_t sensor_rho = { .identification = 0, .address = 0x68, .I2C = 0, };
// Sensor (accelerometer + Gyroscope) for measure angle Theta 1
dr_mpu_data_t sensor_theta1 = { .identification = 1, .address = 0x69, .I2C = 0, };
// Sensor (accelerometer + Gyroscope) for measure angle Theta 2
dr_mpu_data_t sensor_theta2 = { .identification = 2, .address = 0x68, .I2C = 1, };
// Sensor (accelerometer + Gyroscope) for measure angle Theta 3
dr_mpu_data_t sensor_theta3 = { .identification = 3, .address = 0x69, .I2C = 1, };

// Filters Instance 
// Kalman Values for Covariance, erro Measure for measure Angle Rho
Kalman_data kalman_0 = { .Q_angle = 0.1f, .Q_bias = 0.01f,     .R_measure = 0.03f,	.angle = 0, .bias = 0 };
// Kalman Values for Covariance, erro Measure for measure Angle Theta 1
Kalman_data kalman_1 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =0.03f, .angle = 0.0f, .bias = 0.0f };
// Kalman Values for Covariance, erro Measure for measure Angle Theta 2
Kalman_data kalman_2 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure = 0.03f,.angle = 0.0f, .bias = 0.0f };
// Kalman Values for Covariance, erro Measure for measure Angle Theta 3
Kalman_data kalman_3 = { .Q_angle = 0.0001f, .Q_bias = 0.01f,  .R_measure = 0.03f, .angle = 0.0f, .bias = 0.0f };
 
/*PD links parameters*/
// PD parameters and signals to control the link rho
Ctrl_Law_angle_data rho = { .identification = 0, .kp_pd = 2.7, .kv_pd = 0.3, .m = 0.2940, .b =
		1.02549, .k = 0.00044956, .upper_limit = rho_angle_max, .lower_limit =
rho_angle_min, .means = 0, .Th_ref = 0 };
// PD parameters and signals to control the link Theta 1
Ctrl_Law_angle_data theta1 = { .identification = 1, .kp_pd = 2.7, .kv_pd = 0.3, .m = 0.2390,
		.b = 0.98597, .k = 0.001232, .upper_limit = theta1_angle_max,
		.lower_limit = theta1_angle_min, .means = 1.3065, .Th_ref = 0 };
// PD parameters and signals to control the link Theta 2
Ctrl_Law_angle_data theta2 = { .identification = 2, .kp_pd = 2.7, .kv_pd = 0.3, .m = 0.1440,
		.b = 0.951269, .k = 0.009040, .upper_limit = theta2_angle_min,
		.lower_limit = theta2_angle_max, .means = 1.9899, .Th_ref = 0 };
// PD parameters and signals to control the link Theta 3
Ctrl_Law_angle_data theta3 =
		{ .identification = 3, .kp_pd = 5, .kv_pd = 0.6, .m = 0.072, .b =
				1.209418, .k = 0.006143, .upper_limit =
		theta3_angle_max, .lower_limit = theta3_angle_min, .means = 1.3718,
				.Th_ref = 0 };

// References Instance
Reference_data Circle = { .ref_time = pi_2 }, Triangle, Straight, Saw;

/* Funtions Prototype */
/* Configure pins interruptions */
void DR_debug_pin();
/* Interrupt routine in 100Hz for control law*/	
//void DR_interrupt_100Hz();
/*Interrupt for read sensors, execute control law and drive motors*/
void Interruption_Program(); 
/*Update motor Values with new u[k]*/
void Varredura_Motores(); 
/*Calculate Open Loop control Law for all links*/
void Calculate_OL_all_links(Reference_data *Trajectory);
/*Calculate PD control Law for all links*/
void Calculate_PD_all_links(Reference_data *Trajectory);
/* All sensor update for Control Law */
void Varredura_Sensores();
/* Update Actual Value of angle link and update *links.Th*/
void Angles_update(int sensor);

void DR_interrupt_100Hz(){
     // Desativar a flag
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
    // Calculate time interruption
    DR_tick_stop(false);
    Interruption_Program();
    DR_tick_start();

}

/* Global Variables*/
uint_fast8_t status_flag_uart = 0;
uint_fast8_t led_change = 0;
/* Vaiables for Debug */
uint_fast16_t Initial_Rho = 4500;
uint_fast16_t Initial_Theta1 = 4000;
uint_fast16_t Initial_Theta2 = 6000;
uint_fast16_t Initial_Theta3 = 6000;
int main(void)
{	WDT_A_holdTimer();

	/* Pin Config */
	DR_leds_sw_pin();
	DR_uart_pin();
	DR_i2c_pin();		// I2C0: 1.6(SDA) e 1.7(SCL) e I2C1: 6.4(SDA) e 6.5(SCL) 
	DR_pwm_pin();		// PWM: 2.4 (PM_0.1), 2.5 (PM_0.2), 2.6 (PM_0.3), 2.7 (PM_0.4)
	DR_debug_pin();		
	
	/* Peripherals Config */
	DR_uart_config(true);	  // Uart config in 115200 Kbps
	DR_t32_config_Hz(0,100);  // Interruption config in 100 Hz
	DR_i2c_config(0);		  // I2C0 Comunication in 400KHz
	DR_i2c_config(1);		  // I2C1 Comunication in 400KHz
	DR_pwm_config(&PWM_Rho);	  // PWM_rho in 50hz in 60000 resolution
	DR_pwm_config(&PWM_Theta1);	  // PWM_theta1 in 50hz in 60000 resolution
	DR_pwm_config(&PWM_Theta2);	  // PWM_theta2 in 50hz in 60000 resolution
	DR_pwm_config(&PWM_Theta3);	  // PWM_theta3 in 50hz in 60000 resolution

	/* Program initialization */
	DR_leds_init();
	DR_uart_init();

	// Sensor Initialization 
	int16_t status_init;
	DR_mpu6050_ligar(1000);		//6.1 MPU6050 Vcc pin
	status_init = DR_mpu6050_init(&sensor_rho);
	if (status_init != 1)
		printf("\n\rIncorrect startup in rho, code(%d) ", status_init);
	printf("Rho sensor initialized (ok!) ");
	
	status_init = DR_mpu6050_init(&sensor_theta1);
	if (status_init != 1)
		printf("\n\rIncorrect startup in theta1, code(%d) ", status_init);
	printf("Theta 1 sensor initialized (ok!) ");
	
	status_init = DR_mpu6050_init(&sensor_theta2);
	if (status_init != 1)
		printf("\n\rIncorrect startup in theta2, code(%d) ", status_init);
	printf("Theta 2 sensor initialized (ok!) ");
	
	status_init = DR_mpu6050_init(&sensor_theta3);
	if (status_init != 1)
		printf("\n\rIncorrect startup in theta3, code(%d) ", status_init);
	printf("Theta 3 sensor initialized (ok!) ");

	DR_leds_alterar(1);

	// PWM Initialization
	DR_pwm_init(&PWM_Rho, Initial_Rho); // initialize in 10%
	DR_pwm_init(&PWM_Theta1, Initial_Theta1); // initialize in 10%
	DR_pwm_init(&PWM_Theta2, Initial_Theta2); // initialize in 10%
	DR_pwm_init(&PWM_Theta3, Initial_Theta3); // initialize in 10%
	
	/* Interrupt Config */
	DR_uart_interrupt_receive();	// interrupt from input console
	DR_t32_interrupt_init(0,DR_interrupt_100Hz);
	DR_interrupt_on();

	while (1)
	{
		DR_leds_alterar(led_change);
		DR_delay_s(1);
		 DR_PWM_setDuty(&PWM_Rho, Initial_Rho);
		    DR_PWM_setDuty(&PWM_Theta1, Initial_Theta1);
		    DR_PWM_setDuty(&PWM_Theta2, Initial_Theta2);
		    DR_PWM_setDuty(&PWM_Theta3, Initial_Theta3);

		if(status_flag_uart == 1){
			led_change = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\r--Ledupdate(%d)",led_change);
			status_flag_uart = 0;
		}

	}

}
/*Interrupt for read sensors, execute control law and drive motors*/
void Interruption_Program(){
//	References_Circle(&Circle); // 3.29ms
////	References_Saw(&Saw);
//	Inverse_Kinematic(&Circle); // 5.92 - 6.20ms
//	Varredura_Sensores(); //  2.96ms
//	Calculate_PD_all_links(&Circle);
////	Calculate_OL_all_links(&Circle);// 5.92 - 6.20ms
//	Varredura_Motores(); //5.92 - 6.20ms
    DR_PWM_setDuty(&PWM_Rho, Initial_Rho);
    DR_PWM_setDuty(&PWM_Theta1, Initial_Theta1);
    DR_PWM_setDuty(&PWM_Theta2, Initial_Theta2);
    DR_PWM_setDuty(&PWM_Theta3, Initial_Theta3);

}

/*Update motor Values with new u[k]*/
void Varredura_Motores() {
// Atualizar os Motores com os valores de u[k] 
	setPosition_ServoMotor(&PWM_Rho,rho.u);
	setPosition_ServoMotor(&PWM_Theta1,theta1.u);
	setPosition_ServoMotor(&PWM_Theta2,theta2.u);
	setPosition_ServoMotor(&PWM_Theta3,theta3.u);
}
/*Calculate Open Loop control Law for all links*/
void Calculate_OL_all_links(Reference_data *Trajectory){
	Open_Loop_Control_Law(&rho,Trajectory->rho);
	Open_Loop_Control_Law(&theta1,Trajectory->theta1);
	Open_Loop_Control_Law(&theta2,Trajectory->theta2);
	Open_Loop_Control_Law(&theta3,Trajectory->theta3);
}
/*Calculate PD control Law for all links*/
void Calculate_PD_all_links(Reference_data *Trajectory){
	Pd_Control_Law(&rho,Trajectory->rho);
	Pd_Control_Law(&theta1,Trajectory->theta1);
	Pd_Control_Law(&theta2,Trajectory->theta2);
	Pd_Control_Law(&theta3,Trajectory->theta3);
}
/* All sensor update for Control Law */
void Varredura_Sensores(){
    int j;
	for (j = 0; j < 4; j++) {
		Angles_update(j);
	}
}
/* Update Actual Value of angle link and update *links.Th*/
void Angles_update(int sensor) {
	if (sensor == 0) { 
		/* Angle Rho (link Base)*/
		DR_mpu6050_atualizar(&sensor_rho); // Update Gyro and accelerometers values
		sensor_rho.ang_gyro = -sensor_rho.gz * 1.3323e-04f; // Calculate gyro for kalman Filter (put in float!)
		rho.estimativa_motor = (getPosition_ServoMotor(&PWM_Rho));	// Update from internal model estimative.
		
		// Calculate the angle using a Kalman filter
		sensor_rho.ang_updated = getAngle(&kalman_0, rho.estimativa_motor, sensor_rho.ang_gyro, Ts);
		// Calculate the angle relative the others links
		rho.Th = sensor_theta1.ang_updated - pi_2;

	}
	if (sensor == 1) { 
		/* Angle Theta 1 (Link 1) */
		DR_mpu6050_atualizar(&sensor_theta1); // Update Gyro and accelerometers values
		sensor_theta1.ang_gyro = sensor_theta1.gy * 1.3323e-04f; // Calculate gyro for kalman Filter (put in float!)
		
		float accX = sensor_theta1.ax * g * ACC_RESOLUTION;	// Calculate ax from accelerometer by acc resolution
		float accZ = sensor_theta1.az * g * ACC_RESOLUTION; // Calculate az from accelerometer by acc resolution
		sensor_theta1.ang_pitch = atan2f(-accX, accZ);

		// Calculate the angle using a Kalman filter
		sensor_theta1.ang_updated = -getAngle(&kalman_1, sensor_theta1.ang_pitch, sensor_theta1.ang_gyro, Ts);
		// Calculate the angle relative the others links
		theta1.Th = sensor_theta1.ang_updated;
	}
	if (sensor == 2) {
	/* Angle Theta 2 (Link 2) */
		DR_mpu6050_atualizar(&sensor_theta2); // Update Gyro and accelerometers values
		sensor_theta2.ang_gyro = sensor_theta2.gz * 1.3323e-04f; // Calculate gyro for kalman Filter (put in float!)
		
		float accX = sensor_theta2.ax * g * ACC_RESOLUTION;	// Calculate ax from accelerometer by acc resolution
		float accY = sensor_theta2.ay * g * ACC_RESOLUTION; // Calculate az from accelerometer by acc resolution
		sensor_theta2.ang_pitch = atan2f(accX, accY);

		// Calculate the angle using a Kalman filter
		sensor_theta2.ang_updated = getAngle(&kalman_2, sensor_theta2.ang_pitch, sensor_theta2.ang_gyro, Ts);
		// Calculate the angle relative the others links
		theta2.Th = (sensor_theta2.ang_updated + pi_2) - theta1.Th;
	}
	if (sensor == 3) {
	/* Angle Theta 3 (Link 3) */
		DR_mpu6050_atualizar(&sensor_theta3); // Update Gyro and accelerometers values
		sensor_theta3.ang_gyro = sensor_theta3.gz * 1.3323e-04f; // Calculate gyro for kalman Filter (put in float!)
		
		float accX = sensor_theta3.ax * g * ACC_RESOLUTION;	// Calculate ax from accelerometer by acc resolution
		float accY = sensor_theta3.ay * g * ACC_RESOLUTION; // Calculate az from accelerometer by acc resolution
		sensor_theta3.ang_pitch = atan2f(accX, accY);

		// Calculate the angle using a Kalman filter
		sensor_theta3.ang_updated = getAngle(&kalman_3, sensor_theta3.ang_pitch, sensor_theta3.ang_gyro, Ts);
		// Calculate the angle relative the others links
		theta3.Th = (sensor_theta2.ang_updated + pi_2) - theta1.Th- theta2.Th;
	}
}

void DR_debug_pin(){
	/* Pino qualquer como saida para auxiliar o Debug */
	// GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
}
/* Interrupt routine in 100Hz for control law*/	

void EUSCIA0_IRQHandler(void)
{
	UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
	status_flag_uart = 1;
}
