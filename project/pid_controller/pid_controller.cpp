/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   kp = Kpi;
   ki = Kii;
   kd = Kdi;
   omax = output_lim_maxi;
   omin = output_lim_mini;
   err_p = 0.0;
   err_pp = 0.0;
   err_i = 0.0;
   err_d = 0.0;
   dt = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   if (dt > 0) {
      err_pp = err_p;
      err_p = cte;
      err_i += err_p * dt;
      err_d = (err_p - err_pp) / dt;
   }
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    control = kp * err_p + ki * err_i + kd * err_d;
    control = max(min(control, omax), omin);
    cout << "control: " << control << "\t" << err_p * kp << "\t" << err_i * ki << "\t" << err_d * kd << endl;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   dt = new_delta_time;
   cout << "dt: " << dt << endl;
   return dt;
}