/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"

#include <math.h>

#include <iostream>
#include <numeric>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
    /**
     * Initialize PID coefficients (and errors, if needed)
     **/
    k_proportional = Kpi;
    k_derivative = Kdi;
    k_integral = Kii;

    e_proportional = 0.0;
    e_derivative = 0.0;
    e_integral = 0.0;

    output_lim_max = output_lim_maxi;
    output_lim_min = output_lim_mini;
}

void PID::UpdateError(double cte) {
    /**
     * Update PID errors based on cte.
     **/
    e_derivative = (cte - e_proportional) / delta_time;
    e_proportional = cte;
    e_integral += cte * delta_time;
}

double PID::TotalError() {
    /**
     * Calculate and return the total error
     * The code should return a value in the interval [output_lim_mini,
     * output_lim_maxi]
     */

    double control = k_proportional * e_proportional + k_derivative * e_derivative + k_integral * e_integral;
    if (control > output_lim_max)
        control = output_lim_max;
    else if (control < output_lim_min)
        control = output_lim_min;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    /**
     * Update the delta time with new value
     */
    if (new_delta_time <= 0.0) {
        throw std::invalid_argument("Delta time must be a positive value.");
    }
    delta_time = new_delta_time;
}