/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CTRL_ROLL_H
#define CTRL_ROLL_H

#include <math.h>

// Lateral controller for hummingbird
class CtrlRollPD {
  public:
    float phi_d2;
    float phi_d3;
    float phi_dot_d2;
    float phi_dot_d3;
    
    CtrlRollPD() {  
    }

    void init() {
      // persistent variables
      phi_d2 = 0.0;
      phi_d3 = 0.0;
      phi_dot_d2 = 0.0;
      phi_dot_d3 = 0.0;
    }

    void update(SensorUtilities &sensors, 
                MotorUtilities &rotors, 
                float Ts) {

      // compute phi/psi and phi_dot/psi_dot (with quadratic prediction)
      float phi_d1 = sensors.roll;
      float phi = 3*phi_d1 - 3*phi_d2 + phi_d3;
      float phi_dot_d1 = (phi_d1 - phi_d2) / Ts;
      float phi_dot = 3*phi_dot_d1 - 3*phi_dot_d2 + phi_dot_d3;
      float error_phi = phi_ref - phi;

      float force_fl = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2)  / P.ellT;
      float torque = kp_phi * error_phi - kd_phi * phi_dot;
      float force = force_fl; 
      
      float left_pwm = (force+torque/P.d)/(2.0*km);
      float right_pwm = (force-torque/P.d)/(2.0*km);
      rotors.update(left_pwm, right_pwm); 

      phi_d3 = phi_d2;
      phi_d2 = phi_d1;
      phi_d1 = phi;
      phi_dot_d3 = phi_dot_d2;
      phi_dot_d2 = phi_dot_d1; 
    }
};

#endif 
