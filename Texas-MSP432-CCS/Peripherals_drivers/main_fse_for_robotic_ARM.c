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
#include <arm_math.h>
#include <arm_const_structs.h>

/* Misc. definitions. */
#define PI               3.14159265358979f

uint8_t RXData[2];
uint8_t contador = 1;
// Vari�veis relacionadas aos filtro
Luenberger_data luenberger_0 = { .pole1 = 0.1f, .pole2 = 0.2f };
// Instancia dos Filtros
Kalman_data kalman_0 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
                                 0.03f,
                         .angle = 0.0f, .bias = 0.0f };
Kalman_data kalman_1 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
                                 0.03f,
                         .angle = 0.0f, .bias = 0.0f };
Kalman_data kalman_2 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
                                 0.03f,
                         .angle = 0.0f, .bias = 0.0f };
Kalman_data kalman_3 = { .Q_angle = 0.0001f, .Q_bias = 0.01f,
                         .R_measure = 0.03f, .angle = 0.0f, .bias = 0.0f };

// Vari�veis relacionadas aos �ngulos dos elos e controlador PD

Ctrl_Law_angle_data theta1 = { .identification = 1.0f, .kp_pd = 2.7f, .kv_pd =
                                       0.3f,
                               .m = 0.2390f, .b = 0.98597f, .k = 0.001232f,
                               .upper_limit = 1.972f, .lower_limit = 0.562f,
                               .means = 1.3065f, .Th_ref = 0.0f };

// Defini��o do endere�amento dos 4 sensores
dr_mpu_data_t sensor_rho = { .identification = 0, .address = 0x68, .I2C = 0, };
dr_mpu_data_t sensor_theta1 =
        { .identification = 1, .address = 0x69, .I2C = 0, };
dr_mpu_data_t sensor_theta2 =
        { .identification = 2, .address = 0x68, .I2C = 2, };
dr_mpu_data_t sensor_theta3 =
        { .identification = 3, .address = 0x69, .I2C = 2, };
// Vari�veis para instancias as refer�ncias
Reference_data Circle = { .ref_time = pi_2 }, Triangle, Straight, Saw;

float angulos_atualizados[4];

uint_fast8_t status_flag_uart = 0;
uint_fast8_t status_flag_angle_update = 0;
uint_fast8_t status_flag_print = 0;
uint_fast16_t status_flag_diagnostic = 0;
/* For Debug graph*/
uint8_t RXData[2];
uint8_t leitura[14];
float magCalibration[3];
float magValue[3];
float magbias[3];
typedef struct
{
    float last_angle;
    float atual_angle;
    float velocidade_ang;
    float Calibrationx;
    float Calibrationy;
    float Calibrationz;

} filterSE;

filterSE Gyroscope;
uint32_t time = 0;
bool aquisition_Data_Start = 0;

//  Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192.
#define WINDOW_LENGTH 128
float data_realtime = 0;
uint16_t contador_wave = 0;
float window_data[WINDOW_LENGTH];
uint16_t contador_window = 0;
uint16_t fftSize = WINDOW_LENGTH;

volatile arm_status status;
float a = -0.1;
float c = 6;
double tempo_processamento = 0;
float32_t sinal_filtrado;
float32_t data_input[WINDOW_LENGTH * 2];
uint32_t ifftFlag = 0;
float32_t fft_results[WINDOW_LENGTH];
float32_t fft_results_abs[WINDOW_LENGTH];
float32_t fft_w_sigmoid[WINDOW_LENGTH];
float32_t ifft_results[WINDOW_LENGTH];
float32_t eq_sigmoid[WINDOW_LENGTH];

float sinal_n_filtrado;

void DR_Gyroscope_calibrate(dr_mpu_data_t *sensor, filterSE *Gyroscope)
{
    int i = 100;
    int Max_Value[3];
    int Min_Value[3];

    DR_mpu9250_atualizar(&sensor_rho);

    Max_Value[0] = sensor->gx;
    Min_Value[0] = sensor->gx;
    Max_Value[1] = sensor->gy;
    Min_Value[1] = sensor->gy;
    Max_Value[2] = sensor->gz;
    Min_Value[2] = sensor->gz;

    while (i > 0)
    {
        DR_mpu9250_atualizar(&sensor_rho);
        // Higher and lower X-axis Value
        if (sensor->gx > Max_Value[0])
        {
            Max_Value[0] = sensor->gx;
        }
        if (sensor->gx < Min_Value[0])
        {
            Min_Value[0] = sensor->gx;
        }
        // Higher and lower Y-axis Value
        if (sensor->gy > Max_Value[1])
        {
            Max_Value[1] = sensor->gy;
        }
        if (sensor->gy < Min_Value[1])
        {
            Min_Value[1] = sensor->gy;
        }
        // Higher and lower Z-axis Value
        if (sensor->gz > Max_Value[2])
        {
            Max_Value[2] = sensor->gz;
        }
        if (sensor->gz < Min_Value[2])
        {
            Min_Value[2] = sensor->gz;
        }
        DR_delay_k(1);
        i--;
    }
    Gyroscope->Calibrationx = (Max_Value[0] + Min_Value[0]) / 2; // Offset  x-axis
    Gyroscope->Calibrationy = (Max_Value[1] + Min_Value[1]) / 2; // Offset  y-axis
    Gyroscope->Calibrationz = (Max_Value[2] + Min_Value[2]) / 2; // Offset  z-axis
}
void DR_aquisition_dados()
{
    if (time == 0)
    {
        Dr_clc_RGB_red;
        Dr_clc_RGB_blue;
        Dr_set_RGB_green;
    }

    printf("\n\r%d %.6f ", time, Gyroscope.velocidade_ang);

    if (time == 6000)
    {
        Interrupt_disableInterrupt(INT_T32_INT1);
        SysTick_disableInterrupt();
        Dr_clc_RGB_green;
        Dr_set_RGB_blue;
    }
    else
    {
        time++;
    }
}
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

int DR_angles_update(uint16_t n_sensor)
{
    if (n_sensor == 0)
    {
        int status_atualizar = DR_mpu6050_atualizar(&sensor_theta1);
    }
    if (n_sensor == 1)
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

        // Atualiza��o para o Filtro de Subrata��o Espectral
        Gyroscope.velocidade_ang = (sensor_rho.gz - Gyroscope.Calibrationz)
                * 1.3323e-04;
        Gyroscope.last_angle = Gyroscope.atual_angle;
        Gyroscope.atual_angle = Gyroscope.last_angle
                + Gyroscope.velocidade_ang * Ts;
        if (aquisition_Data_Start)
            DR_aquisition_dados();

        return 1;
    }
    if (n_sensor == 2)
    {
        float accX = sensor_rho.mx - sensor_rho.mag_offset_x; // Magnetometer X-axis // Testar multiplicar por 0.1
        float accY = sensor_rho.my - sensor_rho.mag_offset_y; // Magnetometer Y-axis // Testar multiplicar por 0.1

        float gyroZ = sinal_filtrado * 1.3323e-04f; // Gyroscope Z euler angle

        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope

        float pitch = atan2f(accY, accX); // calculate angle Z from Magnetometer
        // -y/x n funcionou
        sensor_rho.ang_pitch = pitch; // Store
        // obs test the movement in a 180 degree in relation the angle of calibration
        float Kal_Angle = getAngle(&kalman_0, pitch, gyroZ, Ts);
        sensor_rho.ang_updated = Kal_Angle;
        return -3;
    }
    if (n_sensor == 3)
    {
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = sinal_filtrado; // calculate angle Z from Magnetometer
        // -y/x n funcionou
        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope
        // obs test the movement in a 180 degree in relation the angle of calibration
        float Kal_Angle = getAngle(&kalman_0, pitch, gyroZ, Ts);
        sensor_rho.ang_updated = Kal_Angle;
    }
    if (n_sensor == 4)
    {
        float gyroZ = sensor_rho.gz * 1.3323e-04f; // Gyroscope Z euler angle
        float pitch = sinal_filtrado; // calculate angle Z from Magnetometer
        // -y/x n funcionou
        sensor_rho.ang_pitch = pitch; // Store
        sensor_rho.ang_gyro = gyroZ; // store Z euler angle gyroscope
        // obs test the movement in a 180 degree in relation the angle of calibration
        sensor_rho.ang_Kalman = getAngle(&kalman_0, pitch, gyroZ, Ts);
        sensor_rho.ang_Luenberger = getAngle_Luen2(&luenberger_0, pitch, gyroZ,
        Ts);
//        sensor_rho.ang_updated = Luen_Angle;
    }
    else
    {
        return 0;
    }
    return 1;
}

void interrupt_angles()
{   // Desativar a flag
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
    DR_tick_start();
    DR_mpu9250_atualizar(&sensor_rho);
//    sinal_n_filtrado = sensor_rho.gz;
    float accX = sensor_rho.mx - sensor_rho.mag_offset_x; // Magnetometer X-axis // Testar multiplicar por 0.1
    float accY = sensor_rho.my - sensor_rho.mag_offset_y; // Magnetometer Y-axis // Testar multiplicar por 0.1
    sinal_n_filtrado = atan2f(accY, accX);

    if (contador_window > WINDOW_LENGTH - 1)
    {
        contador_window = 0;
//        window_data[contador_window] = sensor_rho.gz;
        window_data[contador_window] = sinal_n_filtrado;
        sinal_filtrado = calculatefft(contador_window);
//        DR_angles_update(2);    // Calculate Kalman Filter
        DR_angles_update(4);
        contador_window++;
    }
    else
    {
//        window_data[contador_window] = sensor_rho.gz;
        window_data[contador_window] = sinal_n_filtrado;
        sinal_filtrado = calculatefft(contador_window);
//        DR_angles_update(2);    // Calculate Kalman Filter
        DR_angles_update(4);
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

    /* Peripherals Config */
    DR_uart_config(true);
    DR_i2c_config(0);
    DR_t32_config_Hz(0, 100);

    /* Inicializar Programas*/
    DR_leds_init();
    DR_uart_init();
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
    DR_Gyroscope_calibrate(&sensor_rho, &Gyroscope);
    Dr_set_RGB_blue;

    printf("\n\rSensores Calibrados");
    printf("\n\rOffsetX : %.4f", sensor_rho.mag_offset_x);
    printf("\n\rOffsetY : %.4f", sensor_rho.mag_offset_y);
    printf("\n\rOffsetZ : %.4f", sensor_rho.mag_offset_z);
    printf("\n\rOffset(GyroX) : %.4f", Gyroscope.Calibrationx);
    printf("\n\rOffset(GyroY) : %.4f", Gyroscope.Calibrationy);
    printf("\n\rOffset(GyroZ) : %.4f", Gyroscope.Calibrationz);

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
    contador = UART_receiveData(EUSCI_A0_BASE) - 48;
    printf("\n\n\r--Ledupdate(%d)", contador);
    if (contador == 1)
    { /*Leitura do endereço*/
        aquisition_Data_Start = 1;
    }
    if (contador == 2)
    { /*Leitura acc e gyroscope*/
        //Recalcular os polos
        setPoles_Luen(&luenberger_0, Ts);

    }
    if (contador == 3)
    { /* Leitura Magnetometro calibração*/

    }
    if (contador == 4)
    { /* Leitura Magnetometro data*/

    }

}
