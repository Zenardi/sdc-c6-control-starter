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
    // Store gains and output clamp limits
    _kpi = Kpi;
    _kii = Kii;
    _kdi = Kdi;
    _output_lim_maxi = output_lim_maxi;
    _output_lim_mini = output_lim_mini;

    // Initialise all error accumulators to zero
    _p_error = 0.0;
    _i_error = 0.0;
    _d_error = 0.0;
    _new_delta_time = 0.0;
}


void PID::UpdateError(double cte) {
    // Save previous proportional error to compute derivative
    double previous_p_error = _p_error;

    // P error: current cross-track error
    _p_error = cte;

    // Guard: use dt=1 on first call (delta_time not yet set) to avoid divide-by-zero.
    // Also cap dt to 0.5 s: a very large dt (e.g. when CARLA renders a slow frame)
    // amplifies d_error = Δe/dt, which can saturate the output in the wrong direction.
    double dt = (_new_delta_time > 0.0) ? _new_delta_time : 1.0;
    if (dt > 0.5) dt = 0.5;

    // D error: rate of change of CTE (smooths correction, reduces overshoot)
    _d_error = (cte - previous_p_error) / dt;

    // I error: accumulated CTE * dt (corrects persistent steady-state offset)
    _i_error += cte * dt;
}

double PID::TotalError() {
    // PID output: negative signs because the controller opposes the error
    double control = (-_kpi * _p_error)
                   - (_kii * _i_error)
                   - (_kdi * _d_error);

    // Clamp output to actuator limits
    if (control > _output_lim_maxi) {
        control = _output_lim_maxi;
    } else if (control < _output_lim_mini) {
        control = _output_lim_mini;
    }

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    // Store elapsed time for use in UpdateError's derivative/integral terms
    _new_delta_time = new_delta_time;
    return 0;  // return value is not used by main.cpp
}