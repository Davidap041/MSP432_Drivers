#include "Luenberger.h"

float getAngle_Luen(Luenberger_data *luenberger, float newAngle, float newRate,
                     float dt)
{
    // Store previous values
    luenberger->angle_prev = luenberger->angle;
    luenberger->bias_prev = luenberger->bias;
    // Calculate residues in velocity and position
    luenberger->rate = newRate - luenberger->bias;
    luenberger->residue = newAngle - luenberger->angle;
    // Estimate Angle and Velocity for next iteration
    luenberger->angle = luenberger->angle_prev + luenberger->L1_ * luenberger->residue
            + dt * luenberger->rate;
    luenberger->bias += luenberger->L2_ * luenberger->residue;

    return luenberger->angle_prev;
}

void setPoles_Luen(Luenberger_data *luenberger, float dt)
{
    luenberger->L1_ = 2 - luenberger->pole1 - luenberger->pole2;
    float fs = 1 / dt;
    luenberger->L2_ = (1 - luenberger->L1_
            - luenberger->pole2 * luenberger->pole1) * fs;
}

