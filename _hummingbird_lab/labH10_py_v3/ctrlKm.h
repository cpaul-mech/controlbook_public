/**
 * \file sensors.h
 * \author Randy Beard <beard@byu.edu>
 *
 * class to implement controller
 */

#ifndef CTRL_KM_H
#define CTRL_KM_H

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

// state structure contains current estimate of state
struct State {
  float phi=0.0;
  float theta=0.0;
  float psi=0.0;
  float phi_dot=0.0;
  float theta_dot=0.0;
  float psi_dot=0.0;
};

// reference structure the reference signals for psi and theta
struct Reference {
  float theta = 0.0;
  float psi = 0.0;
  float phi = 0.0;  
};

// Lateral controller for hummingbird
class CtrlKm {
  public:    
    CtrlKm() {  
    }

    void init() {
      
    }

    void update(SensorUtilities &sensors, 
                MotorUtilities &rotors, 
                float Ts) {

      float force = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2) / P.ellT;
      float torque = 0.0;

      float left_pwm = (force+torque/P.d)/(2.0*km);
      float right_pwm = (force-torque/P.d)/(2.0*km);
      rotors.update(left_pwm, right_pwm); 
    }  
};

#endif 
