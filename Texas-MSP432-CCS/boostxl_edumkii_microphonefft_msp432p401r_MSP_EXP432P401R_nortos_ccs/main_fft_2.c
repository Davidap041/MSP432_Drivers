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

extern float wave_input1[256];
extern float wave_input2[256];
uint16_t fftSize = SAMPLE_LENGTH;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
volatile arm_status status;
uint16_t define_wave = 0;

float32_t hann[SAMPLE_LENGTH];
float32_t data_array1[SAMPLE_LENGTH];
float32_t data_input[SAMPLE_LENGTH * 2];
float32_t fft_results[SAMPLE_LENGTH];
float32_t ifft_results[SAMPLE_LENGTH];
float32_t fft_results_abs[SAMPLE_LENGTH];
float32_t fft_w_sigmoid[SAMPLE_LENGTH];
float32_t eq_sigmoid[SAMPLE_LENGTH];
float a = -2;
float c = 50;

int main(void)
{
    WDT_A_holdTimer();

    /*
     * Perform a complex FFT on the input samples. The result is calculated
     * in-place and will be stored in the input buffer.
     */
// Initialize Hann Window
    while (1)
    {

        int n;
        for (n = 0; n < SAMPLE_LENGTH; n++)
        {
            data_input[n] = wave_input1[n];
        }
        /* Computer real FFT using the completed data buffer */
        arm_rfft_fast_instance_f32 instance;
        status = arm_rfft_fast_init_f32(&instance, fftSize);
        arm_rfft_fast_f32(&instance, data_input, fft_results, ifftFlag);
        /* Calculate magnitude of FFT complex output */
        arm_abs_f32(fft_results, fft_results_abs, fftSize);
        /*Multiplicate for sigmoide*/
        int k;
        for (k = 0; k < SAMPLE_LENGTH; k++)
        {
            fft_w_sigmoid[k] = fft_results[k]
                    * (1 / (1 + expf(-a * (k - c))));
            eq_sigmoid[k] = 1 / (1 + expf(-a * (k - c)));
        }
        /* Calculate Inverse FFT*/
        arm_rfft_fast_f32(&instance, fft_w_sigmoid, ifft_results, 1);
    }
}
