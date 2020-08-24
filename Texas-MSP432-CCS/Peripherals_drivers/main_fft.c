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
#include "Drivers_Control.h"
#include "mpu6050.h"
#include "Kalman.h"
#include "Control_Law.h"
#include "function.h"

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
#define SAMPLE_LENGTH 64

uint32_t fftSize = SAMPLE_LENGTH;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
volatile arm_status status;

float_t hann[SAMPLE_LENGTH];
int16_t data_array1[SAMPLE_LENGTH];
int16_t data_input[SAMPLE_LENGTH * 2];
int16_t data_output[SAMPLE_LENGTH];

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
        hann[n] = 0.5f - 0.5f * cos((2 * PI * n) / (SAMPLE_LENGTH - 1));
    }
    int i;
    /* Computer real FFT using the completed data buffer */
    for (i = 0; i < 512; i++)
    {
        data_array1[i] = (int16_t) (hann[i] * data_array1[i]);
    }
//    arm_rfft_instance_f32 instance;
//    status = arm_rfft_instance_f32(&instance, fftSize, ifftFlag, doBitReverse);

//    arm_rfft_instance_f32(&instance, data_array1, wave_input);
    /* Calculate magnitude of FFT complex output */
    for (i = 0; i < 1024; i += 2)
    {
        data_output[i / 2] = (int32_t) (sqrtf(
                (wave_input[i] * wave_input[i])
                        + (wave_input[i + 1] * wave_input[i + 1])));
    }

    return 0;
}
