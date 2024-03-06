/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <math.h>

#include "python_tuning.h"

// physical parameters of the system
static struct {
  float m1=0.108862;
  float ell1=0.247;
  float m2=0.4717;
  float ell2=-0.039;
  float m3=.1905;
  float g = 9.81;
  float ellT = 0.29;
  float ell3x=-.007;
  float ell3y=.018;
  float J1x = 0.000189;
  float J1y = 0.001953;
  float J1z = 0.001894;
  float J2x = 0.00231;
  float J2y = 0.003274;
  float J2z = 0.003416;
  float J3x = 0.0002222;
  float J3y = 0.0001956;
  float J3z = 0.000027;
  float d = 0.12;
  float fe = (m1*ell1+m2*ell2)*g/ellT;  
  float force_max = 0.1;
} P;

// reference structure the reference signals for psi and theta
struct Reference {
  float theta = 0.0;
  float psi = 0.0;
  float phi = 0.0;  
};

// Controller to find the motor constant km
class CtrlLonPID {
  public:
    float theta_d2;
    float theta_d3;
    float theta_dot_d2;
    float theta_dot_d3;
    float integrator_theta;
    float error_theta_d1;
    
    CtrlLonPID() {  
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

      // compute theta and theta_dot (with quadratic prediction)
      float theta_d1 = sensors.pitch;
      float theta = 3*theta_d1 - 3*theta_d2 + theta_d3;
      float theta_dot_d1 = (sensors.pitch - theta_d2) / Ts;
      float theta_dot = 3*theta_dot_d1 - 3*theta_dot_d2 + theta_dot_d3;

      // compute feedback linearized force      
      //float force_fl = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2) * cos(theta) / P.ellT;
      float force_fl = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2)  / P.ellT;

      // compute error
      float error_theta = theta_ref - theta;
      
      // update integrator 
      if (ki_theta < 0.0001) {
        integrator_theta += (Ts/2)*(error_theta + error_theta_d1);
      } else {
        integrator_theta = 0.0;
      }

      // pitch control
      float f_tilde = saturate(+ kp_theta * error_theta 
                               + ki_theta * integrator_theta 
                               - kd_theta * theta_dot, 
                         -P.force_max, P.force_max);                        
      float force = force_fl + f_tilde; 
      float torque = 0.0;
      
      // convert force and torque to pwm and send to motors
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
