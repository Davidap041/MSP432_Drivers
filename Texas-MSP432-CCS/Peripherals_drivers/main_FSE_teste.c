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
#define TEMPO_ESTABILIZACAO_MOTOR 200
#define SIZE_VECTOR_SINAIS_MOTOR 13

extern float prbs[13];
extern float calibration_signal[31];
extern float ensaio_signals[13];

// Variables for Luenberger 
Luenberger_data luenberger_0 = { .pole1 = 0.98f, .pole2 = 0.95f };
// Instance for Kalman Filter
Kalman_data kalman_0 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
                                 0.03f,
                         .angle = 0.0f, .bias = 0.0f };

// Defini��o do endere�amento dos 4 sensores
dr_mpu_data_t sensor_rho = { .identification = 0, .address = 0x68, .I2C = 0, };
// Config TA0.1
dr_pwm_parameters PWM_0 = { .identification = 0, .timer = TIMER_A0_BASE,
                            .fast_mode = true, .timer_Prescaler = 64,
                            .timer_Prescaler = 4,
                            .true_Sawtooth_not_triangular = true,
                            .period_count = 60000, //60360 = 50 Hz(fechado, osciloscópio)
                            .pwm_channel = 1, .outputmode =
                            TIMER_A_OUTPUTMODE_RESET_SET };

/* For Debug graph*/
uint8_t count_RX_Buffer = 1;    // count to tranform from UART
uint8_t ensaio_rotina = 1;      // contador do ensaio

uint32_t time_aquisition = 0;
bool aquisition_Data_Start = 0;
uint16_t contador_wait_motor = 0;
uint8_t update_position_servo_value = 0;

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

float Duty_Debug_Regulation = 0.0f;

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

void DR_aquisition_dados(uint16_t n_ensaio)
{
    time_aquisition++;
    if (n_ensaio == 1)
    { /* Ensaio 1 :: Kalman {Magnetometer + Giroscope}
     * 1 - Sinal Magnetometer
     * 2 - Sinal Giroscope
     * 3 - Sinal da Fus�o  (Kalman Angle)
     * */
        printf("\n\r%d %.6f %.6f %.6f ", time_aquisition, sensor_rho.ang_pitch,
               sensor_rho.ang_gyro, sensor_rho.ang_Kalman);
    }
    if (n_ensaio == 2)
    {/*Ensaio 2 :: Kalman {Magnetometer (w/FSE) + Giroscope}
     * 1 - Sinal Magnetometer (antes FSE)
     * 2 - Sinal Magnetometer (depois FSE)
     * 3 - Sinal Giroscope
     * 4 - Sinal da Fus�o (Kalman Angle)*/
        printf("\n\r%d %.6f %.6f %.6f ", time_aquisition, sensor_rho.ang_pitch,
               sensor_rho.ang_gyro, sensor_rho.ang_Kalman);
    }
    if (n_ensaio == 3)
    {
        printf("\n\r%d %.6f ", time_aquisition, sensor_rho.ang_gyro);
    }
    if (n_ensaio == 4)
    {
        printf("\n\r%d %.6f ", time_aquisition, sensor_rho.ang_gyro);
    }
    if (n_ensaio == 5)
    {
        printf("\n\r%d %.6f ", time_aquisition, sensor_rho.ang_gyro);
    }
}
void Update_Servo_Motor(uint16_t position_table)
{
    float Duty_Table_Value;

    Duty_Table_Value = ensaio_signals[position_table];

    setPosition_ServoMotor(&PWM_0, Duty_Table_Value);
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
            DR_aquisition_dados(0);
    }
    if (n_ensaio == 1)
    { /*Kalman {Magnetometer + Giroscope}*/
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = magnetometer_n_filtrado; // calculate angle Z from Magnetometer

        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope

        sensor_rho.ang_Kalman = getAngle(&kalman_0, pitch, gyroZ, Ts);
        sensor_rho.ang_updated = sensor_rho.ang_Kalman;

        if (aquisition_Data_Start)
            DR_aquisition_dados(1);
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
        if (aquisition_Data_Start)
            DR_aquisition_dados(2);

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

        if (aquisition_Data_Start)
            DR_aquisition_dados(3);

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

        if (aquisition_Data_Start)
            DR_aquisition_dados(4);

        return 1;
    }
    if (n_ensaio == 5)
    {
        /* Luenberger e Kalman {Magnetometer(w/FSE) + Giroscope} */
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = magnetomer_w_FSE; // calculate angle Z from Magnetometer

        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope

        sensor_rho.ang_Luenberger = getAngle_Luen(&luenberger_0, pitch, gyroZ,
        Ts);
        sensor_rho.ang_Kalman = getAngle(&kalman_0, pitch, gyroZ, Ts);
        sensor_rho.ang_updated = sensor_rho.ang_Kalman;

        if (aquisition_Data_Start)
            DR_aquisition_dados(5);

        return 1;
    }
    else
    {
        return 0;
    }
}

void interrupt_angles()
{
// --------------------- Desativar a flag -----------------------------------//
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
    DR_tick_start();

//------------------- Update Servo Motor Value --------------------------------//
    // Start Aquisition Data
    if (aquisition_Data_Start)
    {
        Dr_clc_RGB_red;
        Dr_clc_RGB_blue;
        Dr_set_RGB_green;
        if (contador_wait_motor >= TEMPO_ESTABILIZACAO_MOTOR)
        {   // Motor estabilizado e pronto para novo valor
            contador_wait_motor = 0;

            if (update_position_servo_value >= SIZE_VECTOR_SINAIS_MOTOR)
            { //Termino de todas as posi��es do ensaio e inicio do pr�ximo ensaio
                update_position_servo_value = 0;
                if (ensaio_rotina <= 5)
                {
                    ensaio_rotina++;
                    time_aquisition = 0;
                }
                else
                {
                    ensaio_rotina = 1;
                    aquisition_Data_Start = 0;  // Finalizar Data_Aquisition
                    Dr_clc_RGB_green;
                    Dr_set_RGB_blue;

                }
            }
            else
            {   // Update nova posi��o do motor
                update_position_servo_value++;
                Update_Servo_Motor(update_position_servo_value);
            }
        }
        else
        {
            contador_wait_motor++;
        }
    }
//------------------- Update Angle Values -------------------------------------//
    DR_mpu9250_atualizar(&sensor_rho);

    float MagX = sensor_rho.mx - sensor_rho.mag_offset_x; // Magnetometer X-axis // Testar multiplicar por 0.1
    float MagY = sensor_rho.my - sensor_rho.mag_offset_y; // Magnetometer Y-axis // Testar multiplicar por 0.1
    magnetometer_n_filtrado = atan2f(MagY, MagX);

    if (contador_window > WINDOW_LENGTH - 1)
    {
        contador_window = 0;
        window_data[contador_window] = magnetometer_n_filtrado; // Magnetometer
        magnetomer_w_FSE = calculatefft(contador_window);
        DR_angles_update(ensaio_rotina);
        contador_window++;
    }
    else
    {
        window_data[contador_window] = magnetometer_n_filtrado;
        magnetomer_w_FSE = calculatefft(contador_window);
        DR_angles_update(ensaio_rotina);
        contador_window++;
    }

//------------------- Stop Routine -------------------------------------------//
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
    DR_pwm_init(&PWM_0, 4250);
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
//    sensor_rho.mag_offset_x = 225;
//    sensor_rho.mag_offset_y = 294;
//    sensor_rho.mag_offset_z = 256;
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
    {
        aquisition_Data_Start = 1;
    }

}
