/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CTRL_YAW_H
#define CTRL_YAW_H

#include <math.h>

// Lateral controller for hummingbird
class CtrlYawPID {
  public:
    float phi_d2;
    float phi_d3;
    float phi_dot_d2;
    float phi_dot_d3;
    float psi_d2;
    float psi_d3;
    float psi_dot_d2;
    float psi_dot_d3;    
    float integrator_psi;
    float error_psi_d1;    
    
    CtrlYawPID() {  
    }

    void init() {
      // persistent variables
      phi_d2 = 0.0;
      phi_d3 = 0.0;
      phi_dot_d2 = 0.0;
      phi_dot_d3 = 0.0;
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

      float phi_d1 = sensors.roll;
      float phi = 3*phi_d1 - 3*phi_d2 + phi_d3;
      float phi_dot_d1 = (phi_d1 - phi_d2) / Ts;
      float phi_dot = 3*phi_dot_d1 - 3*phi_dot_d2 + phi_dot_d3;

      float psi_d1 = sensors.yaw;
      float psi = 3*psi_d1 - 3*psi_d2 + psi_d3;
      float psi_dot_d1 = (psi_d1 - psi_d2) / Ts;
      float psi_dot = 3*psi_dot_d1 - 3*psi_dot_d2 + psi_dot_d3;      
      float error_psi = psi_ref - psi; 

      if (ki_psi > 0.001) {
        if (psi_dot < 0.1) {
          integrator_psi += (Ts/2)*(error_psi + error_psi_d1);
        }
      } else {
        integrator_psi = 0.0;
      }

      float phi_ref = -(+ kp_psi * error_psi 
                      + ki_psi * integrator_psi 
                      - kd_psi * psi_dot);  

      float error_phi = phi_ref - phi;              
                         
      float force_fl = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2)  / P.ellT;
      float force = force_fl; 
      float torque = kp_phi * error_phi - kd_phi * phi_dot;                                                   
      float left_pwm = (force+torque/P.d)/(2.0*km);
      float right_pwm = (force-torque/P.d)/(2.0*km);
      rotors.update(left_pwm, right_pwm); 

      // update all delayed variables
      phi_d3 = phi_d2;
      phi_d2 = phi_d1;
      phi_d1 = phi;
      phi_dot_d3 = phi_dot_d2;
      phi_dot_d2 = phi_dot_d1;
      
      psi_d3 = psi_d2;
      psi_d2 = psi_d1;
      psi_d1 = psi;
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
