#include "arduino_setup.h"
#include "time_utilities.h"
#include "sensor_utilities.h"
#include "motor_utilities.h"
#include "reference_utilities.h"
#include "ctrlKm.h"
#include "ctrlRollPD.h"
#include "ctrlPitchPID.h"
#include "ctrlYawPID.h"
#include "ctrlFullPID.h"

//=============================================================================
// declare global structures
//=============================================================================

// instantiate structures
Reference reference;

// instantiate classes as global variables
TimeUtilities timing;
SensorUtilities sensors;
MotorUtilities rotors;
ReferenceUtilities signal_generator;
CtrlKm controllerKm;
CtrlRollPD controllerPDroll;
CtrlPitchPID controllerPIDpitch;
CtrlYawPID controllerPIDyaw;
CtrlFullPID controllerPIDfull;

//=============================================================================
// arduino setup function (runs once at start of simulation)
//=============================================================================
void setup()
{
  // start serial communication
  Serial.begin(19200);

  // initialize all classes
  timing.init();  // initialize current time and sample rate
  sensors.init();  // initialize sensors
  controllerKm.init();
  controllerPDroll.init();
  controllerPIDpitch.init();
  controllerPIDyaw.init();
  controllerPIDfull.init();

  // initialize buttons on breakout board and watchdog timers
  initialize_buttons();
}

//=============================================================================
// arduino loop function (loops forever)
//=============================================================================
void loop()
{
  timing.update();  // update current time and sample rate
  sensors.update();  // update sensors

  phi_en = sensors.roll;
  theta_en = sensors.pitch;
  psi_en = sensors.yaw;

  if (controller == controller_km && paused == false) {
    controllerKm.update(sensors, rotors, timing.Ts);
  } else if (controller == controller_roll && paused == false) {
    controllerPDroll.update(sensors, rotors, timing.Ts);
  } else if (controller == controller_pitch && paused == false) {
    controllerPIDpitch.update(sensors, rotors, timing.Ts);    
  } else if (controller == controller_yaw && paused == false) {
    controllerPIDyaw.update(sensors, rotors, timing.Ts);    
  } else if (controller == controller_full && paused == false) {
    controllerPIDfull.update(sensors, rotors, timing.Ts);    
  }

  // zero encoders if zero button pushed
  zeroButton.update();  
  if ( zeroButton.pressed() ) sensors.zero(); 
   
  // calibrate rotors if calibrate button pushed
  calButton.update();
  if ( calButton.pressed() ) rotors.init();
  
  // reset watchdog timer
  wdt_reset();
  digitalWrite(LED_RX, LOW);

  communication_counter++;
  if (communication_counter > communication_limit) {
    communication_counter = 0;
    String encoderSend = String(char(startMarker)) + "F" + String(phi_en) + "T" + String(theta_en) + "S" + String(psi_en) + String(char(endMarker));
    Serial.write(encoderSend.c_str());
  } 
}

void serialEvent() {
  static byte tempBuffer[128];
  static unsigned int bytesRecvd = 0;
  static boolean inProgress = false;
  while (Serial.available()) {
    byte x = Serial.read();
    // Check for start marker
    if (x == startMarker && !inProgress) {
      bytesRecvd = 0;
      inProgress = true;
    }
    // If in progress, read the data
    if (inProgress) {
      if (x == endMarker) {
        inProgress = false;
        parseData(tempBuffer, bytesRecvd);
        bytesRecvd = 0;
      } else if (bytesRecvd < sizeof(tempBuffer)) {
        tempBuffer[bytesRecvd++] = x;
      }
    }
  }
}

void parseData(byte* buffer, unsigned int length) {
  String stringRecvd = "";
  for (byte n=1; n < length; n++) {
    stringRecvd += (char)buffer[n];
  }

  if (stringRecvd.indexOf(char_pause) != -1) {
    rotors.write_to_esc(0.0, 0.0);
    paused = true;
  }

  if (stringRecvd.indexOf(char_play) != -1) {
    rotors.init();
    paused = false;
  }

  if (stringRecvd.indexOf(char_zero) != -1) {
    sensors.zero();
  }

  if (stringRecvd.indexOf(char_km) != -1) {
    int idx1 = stringRecvd.indexOf(char_val1);
    if (idx1 != -1) {
      controller = controller_km;
      km = stringRecvd.substring(idx1+1).toFloat();
    }
  } else if (stringRecvd.indexOf(char_roll) != -1) {
    int idx1 = stringRecvd.indexOf(char_val1);
    int idx2 = stringRecvd.indexOf(char_val2);
    int idx3 = stringRecvd.indexOf(char_val3);
    int idx4 = stringRecvd.indexOf(char_val4);
    int idx5 = stringRecvd.indexOf(char_val5);
    if (idx1 != -1 && idx2 != -1 && idx3 != -1 && idx4 != -1 && idx5 != -1) {
      controller = controller_roll;
      km = stringRecvd.substring(idx1+1,idx2).toFloat();      
      phi_ref = stringRecvd.substring(idx2+1,idx3).toFloat();
      kp_phi = stringRecvd.substring(idx3+1,idx4).toFloat();
      kd_phi = stringRecvd.substring(idx4+1,idx5).toFloat();
    }
  } else if (stringRecvd.indexOf(char_pitch) != -1) {
    int idx1 = stringRecvd.indexOf(char_val1);
    int idx2 = stringRecvd.indexOf(char_val2);
    int idx3 = stringRecvd.indexOf(char_val3);
    int idx4 = stringRecvd.indexOf(char_val4);
    int idx5 = stringRecvd.indexOf(char_val5);
    int idx6 = stringRecvd.indexOf(char_val6);
    if (idx1 != -1 && idx2 != -1 && idx3 != -1 && idx4 != -1 && idx5 != -1 && idx6 != -1) {
      controller = controller_pitch;
      km = stringRecvd.substring(idx1+1,idx2).toFloat();      
      theta_ref = stringRecvd.substring(idx2+1,idx3).toFloat();
      kp_theta = stringRecvd.substring(idx3+1,idx4).toFloat();
      ki_theta = stringRecvd.substring(idx4+1,idx5).toFloat();
      kd_theta = stringRecvd.substring(idx5+1,idx6).toFloat();
    }
  } else if (stringRecvd.indexOf(char_yaw) != -1) {
    int idx1 = stringRecvd.indexOf(char_val1);
    int idx2 = stringRecvd.indexOf(char_val2);
    int idx3 = stringRecvd.indexOf(char_val3);
    int idx4 = stringRecvd.indexOf(char_val4);
    int idx5 = stringRecvd.indexOf(char_val5);
    int idx6 = stringRecvd.indexOf(char_val6);
    if (idx1 != -1 && idx2 != -1 && idx3 != -1 && idx4 != -1 && idx5 != -1 && idx6 != -1) {
      controller = controller_yaw;
      km = stringRecvd.substring(idx1+1,idx2).toFloat();          
      psi_ref = stringRecvd.substring(idx2+1,idx3).toFloat();
      kp_psi = stringRecvd.substring(idx3+1,idx4).toFloat();
      ki_psi = stringRecvd.substring(idx4+1,idx5).toFloat();
      kd_psi = stringRecvd.substring(idx5+1,idx6).toFloat();
    }
  } else if (stringRecvd.indexOf(char_full) != -1) {
    int idx1 = stringRecvd.indexOf(char_val1);
    int idx2 = stringRecvd.indexOf(char_val2);
    int idx3 = stringRecvd.indexOf(char_val3);
    if (idx1 != -1 && idx2 && idx3) {
      controller = controller_full;   
      theta_ref = stringRecvd.substring(idx1+1,idx2).toFloat();
      psi_ref = stringRecvd.substring(idx2+1,idx3).toFloat();
    }
  }
}
