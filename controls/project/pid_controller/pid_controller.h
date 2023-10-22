/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <vector>

class PID {
   public:
    /*
     * Errors
     */
    double e_proportional;
    double e_derivative;
    double e_integral;

    /*
     * Coefficients
     */
    double k_proportional;
    double k_derivative;
    double k_integral;

    /*
     * Output limits
     */
    double output_lim_max;
    double output_lim_min;

    /*
     * Delta time
     */
    double delta_time;

    /*
     * Constructor
     */
    PID();

    /*
     * Destructor.
     */
    virtual ~PID();

    /*
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);

    /*
     * Calculate the total PID error.
     */
    double TotalError();

    /*
     * Update the delta time.
     */
    double UpdateDeltaTime(double new_delta_time);
};

#endif  // PID_CONTROLLER_H
