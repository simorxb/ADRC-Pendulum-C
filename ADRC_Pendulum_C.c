#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define LENGTH 1000
#define TIME_STEP 0.01
#define PI 3.14159265358979323846

struct ADRC_Control {
    const float l1, l2, l3;             // Observer gains
    const float k_p, k_d;               // Controller gains
    const float b0;                     // System parameter
    float z_hat_1, z_hat_2, z_hat_3;    // Observer states
    const float T;                      // Time step (s)
    float u_prev;                       // Control input at previous step
};

struct Pendulum {
    const float m;  // Mass of the pendulum bob (kg)
    const float l;  // Length of the pendulum (m)
    const float k;  // Damping constant (N m s)
    const float g;  // Gravitational constant (m/s^2)
    const float T;  // Time step (s)
    float theta;    // Angle (radians)
    float omega;    // Angular velocity (radians/s)
};

float ADRC_Control_Step(struct ADRC_Control *adrc, float y, float r) {
    /* This function simulates a single step of the ADRC control.
     *
     * Inputs:
     *   adrc: a pointer to an ADRC_Control struct containing the ADRC parameters
     *   y: the output of the system
     *   r: the reference input
     *
     * The function first calculates the observer dynamics. 
     * Then, it updates the observer states using Euler integration.
     * Finally, it calculates the control law.
     *
     * Returns:
     *   The control input u
     */

    // Observer dynamics
    float dz_hat_1 = -adrc->l1 * adrc->z_hat_1 + adrc->z_hat_2 + adrc->l1 * y;
    float dz_hat_2 = -adrc->l2 * adrc->z_hat_1 + adrc->z_hat_3 + adrc->l2 * y + adrc->b0 * adrc->u_prev;
    float dz_hat_3 = -adrc->l3 * adrc->z_hat_1 + adrc->l3 * y;

    // Euler integration of observer
    adrc->z_hat_1 += dz_hat_1 * adrc->T;
    adrc->z_hat_2 += dz_hat_2 * adrc->T;
    adrc->z_hat_3 += dz_hat_3 * adrc->T;

    // Control law
    float u = (adrc->k_p * (r - adrc->z_hat_1) - adrc->k_d * adrc->z_hat_2 - adrc->z_hat_3) / adrc->b0;
    
    // Store control input for next step
    adrc->u_prev = u;

    return u;
}

float Pendulum_Step(struct Pendulum *pendulum, float tau) {
    /* This function simulates a single step of a pendulum's motion.
     *
     * Inputs:
     *   pendulum: a pointer to a Pendulum struct containing the pendulum parameters
     *   tau: the torque applied to the pendulum
     *
     * The function first calculates the angular acceleration (alpha) using the provided dynamics. 
     * Then, it updates the angular velocity and position of the pendulum using Euler integration.
     *
     * Returns:
     *   The updated angle of the pendulum (in radians)
     */
    float alpha;  // Angular acceleration (radians/s^2)

    // Compute angular acceleration using the provided dynamics
    alpha = (tau - pendulum->k * pendulum->omega - pendulum->m * pendulum->g * pendulum->l * sinf(pendulum->theta)) / (pendulum->m * pendulum->l * pendulum->l);

    // Update the angular velocity and position using Euler integration
    pendulum->omega += alpha * pendulum->T;
    pendulum->theta += pendulum->omega * pendulum->T;

    // Return the updated angle of the pendulum
    return pendulum->theta;
}


int main()
{
    // Current simulation time
    float t = 0;

    // Iteration counter
    int i = 0;

    // Setpoint and output of the first control loop
    float command = 0;
    float stp = PI/2;

    // ADRC initialization
    struct ADRC_Control adrc = {180.0, 10800.0, 216000.0, 36.0, 12.0, 2.0, 0.0, 0.0, 0.0, TIME_STEP, 0.0};

    // Pendulum parameters
    struct Pendulum pendulum = {0.5, 1.0, 0.5, 9.81, TIME_STEP, 0.0, 0.0};

    // Open a file for logging simulation data
    FILE *file = fopen("data.txt", "w");

    /* Implement iteration using a while loop */
    while(i < LENGTH)
    {
        // Change setpoint to 180 deg after 5 seconds
        if (t > 5)
        {
            stp = PI;
        }
        
        // Execute the first control loop
        float command = ADRC_Control_Step(&adrc, pendulum.theta, stp);
        Pendulum_Step(&pendulum, command);

        // Log the current time and control loop values to the file
        fprintf(file, "%f %f %f %f %f\n", t, command, pendulum.theta, stp, adrc.z_hat_3);

        // Increment the time and iteration counter
        t = t + TIME_STEP;
        i = i + 1;
    }

    // Close the file and exit the program
    fclose(file);
    exit(0);
}
