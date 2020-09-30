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
#include "Luenberger.h"

/* Misc. definitions. */
#define PI               3.14159265358979f

// Variables for Luenberger 
Luenberger_data luenberger_0 = { .pole1 = 0.98f, .pole2 = 0.95f };
// Instance for Kalman Filter
Kalman_data kalman_0 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
                                 0.03f,
                         .angle = 0.0f, .bias = 0.0f };

// Defini��o do endere�amento dos 4 sensores
dr_mpu_data_t sensor_rho = { .identification = 0, .address = 0x68, .I2C = 0, };
// Config TA0.1
dr_pwm_parameters PWM_0 = { .timer = TIMER_A0_BASE, .fast_mode = true,
                            .timer_Prescaler = 64, .timer_Prescaler = 4,
                            .true_Sawtooth_not_triangular = true,
                            .period_count = 60000, .period_count = 60000, //60360 = 50 Hz(fechado, osciloscópio)
                            .pwm_channel = 1, .outputmode =
                            TIMER_A_OUTPUTMODE_RESET_SET };

/* For Debug graph*/
uint8_t count_RX_Buffer = 1;    // count to tranform from UART

uint32_t time_aquisition = 0;
bool aquisition_Data_Start = 0;

//  Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192.
#define WINDOW_LENGTH 128
float window_data[WINDOW_LENGTH];
uint16_t contador_window = 0;
uint16_t fftSize = WINDOW_LENGTH;

volatile arm_status status;
float a = -0.1;
float c = 6;
double tempo_processamento = 0;
float32_t magnetomer_w_FSE;
float32_t data_input[WINDOW_LENGTH * 2];
uint32_t ifftFlag = 0;
float32_t fft_results[WINDOW_LENGTH];
float32_t fft_results_abs[WINDOW_LENGTH];
float32_t fft_w_sigmoid[WINDOW_LENGTH];
float32_t ifft_results[WINDOW_LENGTH];
float32_t eq_sigmoid[WINDOW_LENGTH];

float magnetometer_n_filtrado;

float32_t calculatefft(uint16_t position)
{
    /* Computer real FFT using the completed data buffer */
    int k;
    for (k = 0; k < WINDOW_LENGTH; k++)
    {
        if (k + position >= WINDOW_LENGTH - 1)
        {
            data_input[k] = window_data[k + position + 1 - WINDOW_LENGTH];
        }
        else
        {
            data_input[k] = window_data[k + position + 1];
        }
    }

    arm_rfft_fast_instance_f32 instance;
    status = arm_rfft_fast_init_f32(&instance, fftSize);
    arm_rfft_fast_f32(&instance, data_input, fft_results, ifftFlag);

    /*Multiplicate for sigmoide*/
    for (k = 0; k < WINDOW_LENGTH; k++)
    {
        fft_w_sigmoid[k] = fft_results[k] * (1 / (1 + expf(-a * (k - c))));
//        eq_sigmoid[k] = 1 / (1 + expf(-a * (k - c)));
    }
    /* Calculate Inverse FFT*/
    arm_rfft_fast_f32(&instance, fft_w_sigmoid, ifft_results, 1);
    return ifft_results[WINDOW_LENGTH - 1];
}

void DR_aquisition_dados()
{
    if (time_aquisition == 0)
    {
        Dr_clc_RGB_red;
        Dr_clc_RGB_blue;
        Dr_set_RGB_green;
    }

    printf("\n\r%d %.6f ", time_aquisition, sensor_rho.ang_gyro);

    if (time_aquisition == 6000)
    {
        Interrupt_disableInterrupt(INT_T32_INT1);
        SysTick_disableInterrupt();
        Dr_clc_RGB_green;
        Dr_set_RGB_blue;
    }
    else
    {
        time_aquisition++;
    }
}

int DR_angles_update(uint16_t n_ensaio)
{
    if (n_ensaio == 0)
    {
        float accX = sensor_rho.mx - sensor_rho.mag_offset_x; // Magnetometer X-axis // Testar multiplicar por 0.1
        float accY = sensor_rho.my - sensor_rho.mag_offset_y; // Magnetometer Y-axis // Testar multiplicar por 0.1

        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle

        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope

        float pitch = atan2f(accY, accX); // calculate angle Z from Magnetometer
        // -y/x n funcionou
        sensor_rho.ang_pitch = pitch; // Store
        // obs test the movement in a 180 degree in relation the angle of calibration
        float Kal_Angle = getAngle(&kalman_0, pitch, gyroZ, Ts);
        sensor_rho.ang_updated = Kal_Angle;

        if (aquisition_Data_Start)
            DR_aquisition_dados();
    }
    if (n_ensaio == 1)
    {   /*Kalman {Magnetometer + Giroscope}*/
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = magnetometer_n_filtrado; // calculate angle Z from Magnetometer
        
        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope
        
        sensor_rho.ang_Kalman = getAngle(&kalman_0, pitch, gyroZ, Ts);
        sensor_rho.ang_updated = sensor_rho.ang_Kalman;

        return 1;
    }
    if (n_ensaio == 2)
    {
        /*Kalman {Magnetometer(w/FSE) + Giroscope}*/
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = magnetomer_w_FSE; // calculate angle Z from Magnetometer
        
        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope
        
        sensor_rho.ang_Kalman = getAngle(&kalman_0, pitch, gyroZ, Ts);
        sensor_rho.ang_updated = sensor_rho.ang_Kalman;

        return 1;
    }
    if (n_ensaio == 3)
    {
        /*Luenberger {Magnetometer + Giroscope}*/
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = magnetometer_n_filtrado; // calculate angle Z from Magnetometer
        
        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope
        
        sensor_rho.ang_Luenberger = getAngle_Luen(&luenberger_0, pitch, gyroZ,
        Ts);
        sensor_rho.ang_updated = sensor_rho.ang_Luenberger;

        return 1;
    }
    if (n_ensaio == 4)
    {   
        /* Luenberger {Magnetometer(w/FSE) + Giroscope} */
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = magnetomer_w_FSE; // calculate angle Z from Magnetometer
        
        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope
        
        sensor_rho.ang_Luenberger = getAngle_Luen(&luenberger_0, pitch, gyroZ,
        Ts);
        sensor_rho.ang_updated = sensor_rho.ang_Luenberger;

        return 1;
    }
    if (n_ensaio == 5){
        /* Luenberger e Kalman {Magnetometer(w/FSE) + Giroscope} */
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = magnetomer_w_FSE; // calculate angle Z from Magnetometer
        
        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope
        
        sensor_rho.ang_Luenberger = getAngle_Luen(&luenberger_0, pitch, gyroZ,
        Ts);
        sensor_rho.ang_Kalman = getAngle(&kalman_0, pitch, gyroZ, Ts);
        return 1;
    }
    else
    {
        return 0;
    }
}

void interrupt_angles()
{   // Desativar a flag
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
    DR_tick_start();
    DR_mpu9250_atualizar(&sensor_rho);

    float MagX = sensor_rho.mx - sensor_rho.mag_offset_x; // Magnetometer X-axis // Testar multiplicar por 0.1
    float MagY = sensor_rho.my - sensor_rho.mag_offset_y; // Magnetometer Y-axis // Testar multiplicar por 0.1
    magnetometer_n_filtrado = atan2f(MagY, MagX);

    if (contador_window > WINDOW_LENGTH - 1)
    {
        contador_window = 0;
        window_data[contador_window] = magnetometer_n_filtrado; // Magnetometer
        magnetomer_w_FSE = calculatefft(contador_window);
        DR_angles_update(5);
        contador_window++;
    }
    else
    {
        window_data[contador_window] = magnetometer_n_filtrado;
        magnetomer_w_FSE = calculatefft(contador_window);
        DR_angles_update(5);
        contador_window++;
    }
    tempo_processamento = DR_tick_stop(false);
}

int main(void)
{
    WDT_A_holdTimer();

    /* Pin Config */
    DR_leds_sw_pin();
    DR_uart_pin();
    DR_i2c_pin();
    DR_pwm_pin();

    /* Peripherals Config */
    DR_uart_config(true);
    DR_i2c_config(0);
    DR_t32_config_Hz(0, 100);
    DR_pwm_config(&PWM_0);

    /* Inicializar Programas*/
    DR_leds_init();
    DR_uart_init();
    DR_pwm_init(&PWM_0, 30000);
    DR_i2c_init(0);
    DR_t32_init(0);

    DR_mpu6050_ligar(10); // I'm not using this now!
    // status_erro = DR_mpu6050_read(sensor_rho.I2C, sensor_rho.address, 0x75, &data) - 10;
    // // while (data != 0x68);

    uint_fast8_t status_init = 0;
    status_init = DR_mpu9250_init(&sensor_rho);
    if (status_init)
    {
        printf("\n\rSensores Inicializados");
        Dr_set_RGB_red;
    }

    // Auto Calibração do Magnetômetro
    DR_magnetometer_calibrate(&sensor_rho);
    DR_Gyroscope_calibrate(&sensor_rho);
    Dr_set_RGB_blue;

    printf("\n\rSensores Calibrados");
    printf("\n\rOffsetX : %.4f", sensor_rho.mag_offset_x);
    printf("\n\rOffsetY : %.4f", sensor_rho.mag_offset_y);
    printf("\n\rOffsetZ : %.4f", sensor_rho.mag_offset_z);
    printf("\n\rOffset(GyroX) : %.4f", sensor_rho.gyro_offset_x);
    printf("\n\rOffset(GyroY) : %.4f", sensor_rho.gyro_offset_y);
    printf("\n\rOffset(GyroZ) : %.4f", sensor_rho.gyro_offset_z);

    /* Interrupt Config */
    DR_uart_interrupt_receive();
    DR_interrupt_on();
    DR_t32_interrupt_init(0, interrupt_angles);

    /* Poles Luenberger */
    setPoles_Luen(&luenberger_0, Ts);
    while (1)
    {

    }
}

void EUSCIA0_IRQHandler(void)
{
    UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    count_RX_Buffer = UART_receiveData(EUSCI_A0_BASE) - 48;
    printf("\n\n\r--Ledupdate(%d)", count_RX_Buffer);
    if (count_RX_Buffer == 1)
    { /*Leitura do endereço*/
        aquisition_Data_Start = 1;
    }
    if (count_RX_Buffer == 2)
    { /*Leitura acc e gyroscope*/
        //Recalcular os polos
        setPoles_Luen(&luenberger_0, Ts);

    }
    if (count_RX_Buffer == 3)
    { /**/
        uint16_t duty_1 = DR_pwm_getDuty(&PWM_0);
        uint16_t period_timer_0 = DR_pwm_getPeriod(&PWM_0);
        if (duty_1 < period_timer_0)
        {
            duty_1 += 10000;
            DR_PWM_setDuty(&PWM_0, duty_1);
        }

    }
    if (count_RX_Buffer == 4)
    { /* Leitura Magnetometro data*/
        uint16_t duty_1 = DR_pwm_getDuty(&PWM_0);
        if (duty_1 > 0)
        {
            duty_1 -= 10000;
            DR_PWM_setDuty(&PWM_0, duty_1);
        }
    }
    if (count_RX_Buffer == 5)
    {
        uint16_t duty_1 = DR_pwm_getDuty(&PWM_0);
        uint16_t period_timer_0 = DR_pwm_getPeriod(&PWM_0);
        printf("\n\r CLK_freq :%d", CS_getSMCLK());
        printf("\n\r Pwm_fast_mode :%x", PWM_0.fast_mode);
        printf("\n\r Pwm_Prescaler :%d", PWM_0.timer_Prescaler);
        printf("\n\r Pwm_Sawtooth :%d", PWM_0.true_Sawtooth_not_triangular);
        printf("\n\r Pwm_Duty1 :%d", duty_1);
        printf("\n\r Pwm_Period0 :%d", period_timer_0);
        printf("\n\r Pwm_freq1 :%.3fHz", DR_pwm_getfreq(&PWM_0));
        printf("\n\r Pwm_duty1 :%.2f%%", DR_pwm_getDuty_percent(&PWM_0));
    }

}
