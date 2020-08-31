//*****************************************************************************
// fft: Qmath signal generator and complex FFT example.
//
// Generate an input signal based on an array of wave descriptors. Each wave
// descriptor is composed of a frequency, amplitude and phase angle. The
// input signal is constructed with a size of SAMPLES and assumes a sample
// frequency defined by SAMPLE_FREQUENCY. The real component of the input
// consists of the summation of all the waves at that time index and the
// imaginary component is set to zero.
// B. Peterson
// Texas Instruments Inc.
// January 2015
//*****************************************************************************
#include "Drivers_Control.h"
#include "mpu6050.h"
#include "Kalman.h"
#include "Control_Law.h"
#include "function.h"
/*other includes*/
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <ti/grlib/grlib.h>
/* Select the global Q value */

/* Include the iqmathlib header files */
#include <ti/devices/msp432p4xx/inc/msp432.h>

#include <arm_math.h>
#include <arm_const_structs.h>

/* Misc. definitions. */
#define PI               3.14159265358979f

/*
 * Specify wave structures that will be used to construct the input signal to
 * the complex FFT function.
 */

/* Calculate the number of wave structures that have been provided. */
#define SAMPLE_LENGTH 256
//#define SMCLK_FREQUENCY     48000000
//#define SAMPLE_FREQUENCY    8000

extern float wave_input1[SAMPLE_LENGTH];
extern float wave_input2[SAMPLE_LENGTH];


volatile arm_status status;


float a = -2;
float c = 10;
double tempo_processamento = 0;

#define WINDOW_LENGTH 32
float data_realtime = 0;
uint16_t contador_wave = 0;
float window_data[WINDOW_LENGTH];
uint16_t contador_window = 0;
uint16_t fftSize = WINDOW_LENGTH;

float32_t data_input[WINDOW_LENGTH * 2];
uint32_t ifftFlag = 0;
float32_t fft_results[WINDOW_LENGTH];
float32_t fft_results_abs[WINDOW_LENGTH];
float32_t fft_w_sigmoid[WINDOW_LENGTH];
float32_t ifft_results[WINDOW_LENGTH];
float32_t eq_sigmoid[WINDOW_LENGTH];


void calculatefft(){
    int n;
            for (n = 0; n < WINDOW_LENGTH; n++)
            {
                data_input[n] = window_data[n];
            }
            /* Computer real FFT using the completed data buffer */
            arm_rfft_fast_instance_f32 instance;
            status = arm_rfft_fast_init_f32(&instance, fftSize);
            arm_rfft_fast_f32(&instance, data_input, fft_results, ifftFlag);
            /* Calculate magnitude of FFT complex output */
            arm_abs_f32(fft_results, fft_results_abs, fftSize);
            /*Multiplicate for sigmoide*/
            int k;
            for (k = 0; k < WINDOW_LENGTH; k++)
            {
                fft_w_sigmoid[k] = fft_results[k] * (1 / (1 + expf(-a * (k - c))));
                eq_sigmoid[k] = 1 / (1 + expf(-a * (k - c)));
            }
            /* Calculate Inverse FFT*/
            arm_rfft_fast_f32(&instance, fft_w_sigmoid, ifft_results, 1);
}
void interrupt_pass_wave()
{   // Desativar a flag
    Timer32_clearInterruptFlag(TIMER32_0_BASE);


    if (contador_wave > SAMPLE_LENGTH - 1)
    {
        contador_wave = 0;
        data_realtime = wave_input2[contador_wave];
        contador_wave++;
    }
    else
    {
        data_realtime = wave_input2[contador_wave];
        contador_wave++;
    }
    if (contador_window > WINDOW_LENGTH-1)
    {
        DR_tick_start();
        calculatefft();
        tempo_processamento = DR_tick_stop(false);
        contador_window = 0;
        contador_window = data_realtime;
        contador_window++;
    }
    else
    {
        window_data[contador_window] = data_realtime;
        contador_window ++;
    }


}

int main(void)
{
    WDT_A_holdTimer();
    DR_t32_config_Hz(0, 100);
    DR_t32_init(0);
    DR_t32_interrupt_init(0, interrupt_pass_wave);

    /*
     * Perform a complex FFT on the input samples. The result is calculated
     * in-place and will be stored in the input buffer.
     */

// Initialize Hann Window
    while (1)
    {


    }
}
