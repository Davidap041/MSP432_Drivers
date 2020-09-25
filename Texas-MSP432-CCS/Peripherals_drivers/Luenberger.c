#include "Luenberger.h"

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle_Luen1(Luenberger_data *luenberger, float newAngle, float newRate,
                     float dt)
{
    // Discrete luenberger filter time update equations - Time Update ("Predict")
    luenberger->angle_prev = luenberger->angle;
    // Update xhat - Project the state ahead
    luenberger->rate = newRate - luenberger->bias;
    luenberger->angle = (1 - luenberger->L1_) * luenberger->angle_prev
            + dt * luenberger->rate;

    luenberger->bias = luenberger->bias
            - luenberger->L2_ * luenberger->angle_prev;
    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    float y = newAngle - luenberger->angle; // Angle difference

    /* Step 6 */
    luenberger->angle += luenberger->L1_ * y;
    luenberger->bias += luenberger->L2_ * y;

    return luenberger->angle;
}
// de acordo com os slides
float getAngle_Luen2(Luenberger_data *luenberger, float newAngle, float newRate,
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

