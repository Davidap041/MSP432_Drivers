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

extern float wave_input[256];
uint16_t fftSize = SAMPLE_LENGTH;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
volatile arm_status status;

float32_t hann[SAMPLE_LENGTH];
float32_t data_array1[SAMPLE_LENGTH];
float32_t data_input[SAMPLE_LENGTH * 2];
float32_t data_output[SAMPLE_LENGTH];
uint32_t color = 0;

int main(void)
{
    WDT_A_holdTimer();

    /*
     * Perform a complex FFT on the input samples. The result is calculated
     * in-place and will be stored in the input buffer.
     */
//    for ()
//    data_array1 =
    /*
     * FFT code here!
     *
     * */

// Initialize Hann Window
    int n;
    for (n = 0; n < SAMPLE_LENGTH; n++)
    {
//         hann[n] = 0.5f - 0.5f * cosf((2 * PI * n) / (SAMPLE_LENGTH - 1));
        data_array1[n] = wave_input[n];
    }
    /* Computer real FFT using the completed data buffer */
    arm_rfft_fast_instance_f32 instance;
    status = arm_rfft_fast_init_f32(&instance, fftSize);

    arm_rfft_fast_f32(&instance, data_array1, data_input, ifftFlag);
    /* Calculate magnitude of FFT complex output */
    arm_abs_f32(data_input, data_output, fftSize);
    float maxValue;
    uint32_t maxIndex = 0;

    arm_max_f32(data_output, fftSize, &maxValue, &maxIndex);

    if (maxIndex <= 64)
    {
        color = ((uint32_t)(0xFF * (maxIndex / 64.0f)) << 8) + 0xFF;
    }
    else if (maxIndex <= 128)
    {
        color = (0xFF - (uint32_t)(0xFF * ((maxIndex - 64) / 64.0f))) + 0xFF00;
    }
    else if (maxIndex <= 192)
    {
        color = ((uint32_t)(0xFF * ((maxIndex - 128) / 64.0f)) << 16) + 0xFF00;
    }
    else
    {
        color = ((0xFF - (uint32_t)(0xFF * ((maxIndex - 192) / 64.0f))) << 8)
                + 0xFF0000;
    }
    return 0;
}
