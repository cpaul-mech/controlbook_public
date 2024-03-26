/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CTRL_PITCH_H
#define CTRL_PITCH_H

#include <math.h>

// Lateral controller for hummingbird
class CtrlPitchPID {
  public:
    float theta_d2;
    float theta_d3;
    float theta_dot_d2;
    float theta_dot_d3;
    float integrator_theta;
    float error_theta_d1; 
    
    CtrlPitchPID() {  
    }

    void init() {
      // persistent variables
      integrator_theta = 0.0;
      theta_d2 = 0.0;
      theta_d3 = 0.0;
      theta_dot_d2 = 0.0;
      theta_dot_d3 = 0.0;
      error_theta_d1 = 0.0;
    }

    void update(SensorUtilities &sensors, 
                MotorUtilities &rotors, 
                float Ts) {

      float theta_d1 = sensors.pitch;
      float theta = 3*theta_d1 - 3*theta_d2 + theta_d3;
      float theta_dot_d1 = (theta_d1 - theta_d2) / Ts;
      float theta_dot = 3*theta_dot_d1 - 3*theta_dot_d2 + theta_dot_d3;
      float error_theta = theta_ref - theta;

      if (ki_theta > 0.001) {
        if (theta_dot < 0.1) {
          integrator_theta += (Ts/2)*(error_theta + error_theta_d1);
        }
      } else {
        integrator_theta = 0.0;
      }

      float force_fl = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2)  / P.ellT;
      float f_tilde = saturate(+ kp_theta * error_theta 
                               + ki_theta * integrator_theta 
                               - kd_theta * theta_dot, 
                         -P.force_max, P.force_max);                        
      float force = force_fl + f_tilde; 
      float torque = 0.0;
                                                
      float left_pwm = (force+torque/P.d)/(2.0*km);
      float right_pwm = (force-torque/P.d)/(2.0*km);
      rotors.update(left_pwm, right_pwm);          
                         
      // update all delayed variables
      theta_d3 = theta_d2;
      theta_d2 = theta_d1;
      theta_dot_d3 = theta_dot_d2;
      theta_dot_d2 = theta_dot_d1;
      error_theta_d1 = error_theta;  
    }

    float saturate(float value, float min_value, float max_value) {
      // Implements the saturation function
      return min(max(min_value, value), max_value);
    }  
};

#endif 
