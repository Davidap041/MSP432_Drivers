#ifndef _Luenberger_h_
#define _Luenberger_h_
typedef struct
{
    float angle; // The angle calculated by the luenberger filter - part of the 2x1 state vector
    float angle_prev;

    float bias; // The gyro bias calculated by the luenberger filter - part of the 2x1 state vector
    float bias_prev;

    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    float residue;

    float pole1;
    float pole2;

    float L1_;
    float L2_;
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
}Luenberger_data;
float getAngle_Luen1(Luenberger_data *luenberger, float newAngle, float newRate,
               float dt);
float getAngle_Luen2(Luenberger_data *luenberger, float newAngle, float newRate,
               float dt);
void setPoles_Luen(Luenberger_data *luenberger, float dt);

#endif
