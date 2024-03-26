/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CTRL_FULL_H
#define CTRL_FULL_H

#include <math.h>

// Lateral controller for hummingbird
class CtrlFullPID {
  public:
    float phi_d2;
    float phi_d3;
    float phi_dot_d2;
    float phi_dot_d3;

    float theta_d2;
    float theta_d3;
    float theta_dot_d2;
    float theta_dot_d3;
    float integrator_theta;
    float error_theta_d1; 

    float psi_d2;
    float psi_d3;
    float psi_dot_d2;
    float psi_dot_d3;    
    float integrator_psi;
    float error_psi_d1;    
    
    CtrlFullPID() {  
    }

    void init() {
      // persistent variables
      phi_d2 = 0.0;
      phi_d3 = 0.0;
      phi_dot_d2 = 0.0;
      phi_dot_d3 = 0.0;

      theta_d2 = 0.0;
      theta_d3 = 0.0;
      theta_dot_d2 = 0.0;
      theta_dot_d3 = 0.0;
      integrator_theta = 0.0;
      error_theta_d1 = 0.0;

      psi_d2 = 0.0;
      psi_d3 = 0.0;
      psi_dot_d2 = 0.0;
      psi_dot_d3 = 0.0; 
      integrator_psi = 0.0;     
      error_psi_d1 = 0.0;      
    }

    void update(SensorUtilities &sensors, 
                MotorUtilities &rotors, 
                float Ts) {

      // compute phi/psi and phi_dot/psi_dot (with quadratic prediction)
      float phi_d1 = sensors.roll;
      float phi = 3*phi_d1 - 3*phi_d2 + phi_d3;
      float phi_dot_d1 = (phi_d1 - phi_d2) / Ts;
      float phi_dot = 3*phi_dot_d1 - 3*phi_dot_d2 + phi_dot_d3;

      float theta_d1 = sensors.pitch;
      float theta = 3*theta_d1 - 3*theta_d2 + theta_d3;
      float theta_dot_d1 = (theta_d1 - theta_d2) / Ts;
      float theta_dot = 3*theta_dot_d1 - 3*theta_dot_d2 + theta_dot_d3;
      float error_theta = theta_ref - theta;

      float psi_d1 = sensors.yaw;
      float psi = 3*psi_d1 - 3*psi_d2 + psi_d3;
      float psi_dot_d1 = (psi_d1 - psi_d2) / Ts;
      float psi_dot = 3*psi_dot_d1 - 3*psi_dot_d2 + psi_dot_d3;
      float error_psi = psi_ref - psi;
      
      if (ki_theta > 0.001) {
        if (theta_dot < 0.1) {
          integrator_theta += (Ts/2)*(error_theta + error_theta_d1);
        }
      } else {
        integrator_theta = 0.0;
      }

      float force_fl = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2) * cos(theta) / P.ellT;
      float f_tilde = saturate(+ kp_theta * error_theta 
                               + ki_theta * integrator_theta 
                               - kd_theta * theta_dot, 
                               -P.force_max, P.force_max);                        
      float force = force_fl + f_tilde; 

      // update integrator 
      if (ki_psi > 0.0) {
        if (psi_dot < 0.1) {
          integrator_psi += (Ts/2)*(error_psi + error_psi_d1);
        }
      }
      else {
        integrator_psi = 0.0;
      }

      float phi_ref =   saturate(+ kp_psi * error_psi 
                                 + ki_psi * integrator_psi 
                                 - kd_psi * psi_dot,
                                 - 30*PI/180, 30*PI/180);                        
                         
      // compute error
      float error_phi = phi_ref - phi; 

      // roll control
      float torque = kp_phi * error_phi - kd_phi * phi_dot;       
      
      // convert force and torque to pwm and send to motors
      float left_pwm = (force+torque/P.d)/(2.0*km);
      float right_pwm = (force-torque/P.d)/(2.0*km);
      rotors.update(left_pwm, right_pwm); 

      // update all delayed variables
      phi_d3 = phi_d2;
      phi_d2 = phi_d1;
      phi_dot_d3 = phi_dot_d2;
      phi_dot_d2 = phi_dot_d1;
      
      theta_d3 = theta_d2;
      theta_d2 = theta_d1;
      theta_dot_d3 = theta_dot_d2;
      theta_dot_d2 = theta_dot_d1;
      error_theta_d1 = error_theta;  
      
      psi_d3 = psi_d2;
      psi_d2 = psi_d1;
      psi_dot_d3 = psi_dot_d2;
      psi_dot_d2 = psi_dot_d1;      
      error_psi_d1 = error_psi; 
    }

    float saturate(float value, float min_value, float max_value) {
      // Implements the saturation function
      return min(max(min_value, value), max_value);
    }  
};

#endif 
