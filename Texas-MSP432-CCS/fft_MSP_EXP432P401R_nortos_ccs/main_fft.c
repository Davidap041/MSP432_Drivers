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
#define GLOBAL_Q    12
/* Include the iqmathlib header files */
#include <ti/iqmathlib/QmathLib.h>
#include <ti/iqmathlib/IQmathLib.h>
#include <ti/devices/msp432p4xx/inc/msp432.h>

/* Specify the sample size and sample frequency. */
#define SAMPLES         64              // power of 2 no larger than 256
#define SAMPLE_FREQ     8192            // no larger than 16384

/* Access the real and imaginary parts of an index into a complex array. */
#define RE(x)           (((x)<<1)+0)    // access real part of index
#define IM(x)           (((x)<<1)+1)    // access imaginary part of index

/*
 * Input and result buffers. These can be viewed in memory or printed by
 * defining ALLOW_PRINTF.
 */
_q qInput[SAMPLES*2];                   // Input buffer of complex values
_q qMag[SAMPLES/2];                     // Magnitude of each frequency result
_q qPhase[SAMPLES/2];                   // Phase of each frequency result

/* Misc. definitions. */
#define PI      3.1415926536

/* Structure that describes a single wave to be used to construct the signal */
typedef struct wave {
    int16_t     frequency;              // Frequency in Hz
    _q          amplitude;              // Amplitude of the signal
    _q          phase;                  // Phase angle in radians
} wave;

/*
 * Specify wave structures that will be used to construct the input signal to
 * the complex FFT function.
 */
const wave signals[] = {
/*   Frequency (Hz)     Magnitude       Phase angle (radians) */
    {128,               _Q(0.5),        _Q(PI/2)},
    {512,               _Q(2.0),        _Q(0)},
    {2048,              _Q(1.333),      _Q(-PI/2)}
};

/* Calculate the number of wave structures that have been provided. */
#define NUM_WAVES       (sizeof(signals)/sizeof(wave))

int main(void)
{
    WDT_A_holdTimer();

    int16_t i, j;                       // loop counters
    _q qWaveCurrentAngle[NUM_WAVES];    // input angles for each signal

    /* Set the initial input angles. */
    for (i = 0; i < NUM_WAVES; i++) {
        qWaveCurrentAngle[i] = signals[i].phase;
    }

    /* Construct the input signal from the wave structures. */
    for (i = 0; i < SAMPLES; i++) {
        qInput[RE(i)] = 0;
        qInput[IM(i)] = 0;
        for (j = 0; j < NUM_WAVES; j++) {
            /*
             * input[RE] += cos(angle)*amplitude
             * angle += 2*pi*freq/sample_freq
             */
            qInput[RE(i)] += _Qmpy(_Qcos(qWaveCurrentAngle[j]), signals[j].amplitude);
            qWaveCurrentAngle[j] += _Qmpy(_Q(2*PI), _Qdiv(signals[j].frequency, SAMPLE_FREQ));
            if (qWaveCurrentAngle[j] > _Q(PI)) {
                qWaveCurrentAngle[j] -= _Q(2*PI);
            }
        }
    }

    /*
     * Perform a complex FFT on the input samples. The result is calculated
     * in-place and will be stored in the input buffer.
     */
    /*
     * FFT code here!
     *
     * */


    return 0;
}
