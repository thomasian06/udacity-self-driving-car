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

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
               double output_lim_mini) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   **/
  this->k_proportional = Kpi;
  this->k_derivative = Kdi;
  this->k_integral = Kii;

  this->e_proportional = 0.0;
  this->e_derivative = 0.0;
  this->e_integral = 0.0;

  this->output_lim_max = output_lim_maxi;
  this->output_lim_min = output_lim_mini;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   **/
  this->e_derivative = (cte - this->e_proportional) / this->delta_time;
  this->e_proportional = cte;
  this->e_integral += cte * this->delta_time;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini,
   * output_lim_maxi]
   */

  double control = this->k_proportional * this->e_proportional +
                   this->k_derivative * this->e_derivative +
                   this->k_integral * this->e_integral;
  if (control > this->output_lim_max)
    control = this->output_lim_max;
  else if (control < this->output_lim_min)
    control = this->output_lim_min;
  return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /**
   * Update the delta time with new value
   */
  this->delta_time = new_delta_time;
}