#include "arduino_setup.h"
#include "time_utilities.h"
#include "sensor_utilities.h"
#include "motor_utilities.h"
#include "reference_utilities.h"
#include "ctrlLonPID.h"

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
CtrlLonPID controller;

//=============================================================================
// arduino setup function (runs once at start of simulation)
//=============================================================================
void setup()
{
  // start serial communication
  Serial.begin(9600);

  // initialize all classes
  timing.init();  // initialize current time and sample rate
  sensors.init();  // initialize sensors
  controller.init();  // initialize controller
  signal_generator.init(15*3.14/180, 0.1, 0.0);

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
  controller.update(sensors, rotors, timing.Ts);  // update controller

  phi_en = sensors.roll;
  theta_en = sensors.pitch;
  psi_en = sensors.yaw;

  // zero encoders if zero button pushed
  zeroButton.update();  
  if ( zeroButton.pressed() ) sensors.zero(); 
   
  // calibrate rotors if calibrate button pushed
  calButton.update();
  if ( calButton.pressed() ) rotors.init();
  
  // reset watchdog timer
  wdt_reset();
  digitalWrite(LED_RX, LOW);
}

void serialEvent() {
  static byte tempBuffer[64];
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
        parseData(tempBuffer, bytesRecvd);  // Ensure parseData is defined or implemented elsewhere
        bytesRecvd = 0;
      } else if (bytesRecvd < sizeof(tempBuffer)) {
        tempBuffer[bytesRecvd++] = x;
      }
    }
  }
}

void parseData(byte* buffer, unsigned int length) {
  String stringRecvd = "";
  for (byte n=1; n < length - 1; n++) {
    stringRecvd += (char)buffer[n];
  }
  int tR_pos = stringRecvd.indexOf('R');
  int km_pos = stringRecvd.indexOf('K');
  int kp_pos = stringRecvd.indexOf('P');
  int ki_pos = stringRecvd.indexOf('I');
  int kd_pos = stringRecvd.indexOf('D');

  if (tR_pos != -1 && km_pos != -1 && kp_pos != -1 && ki_pos != -1 && kd_pos != -1) {
    theta_ref = stringRecvd.substring(tR_pos+1, km_pos).toFloat();
    km = stringRecvd.substring(km_pos+1, kp_pos).toFloat();
    kp_theta = stringRecvd.substring(kp_pos+1, ki_pos).toFloat();
    ki_theta = stringRecvd.substring(ki_pos+1, kd_pos).toFloat();
    kd_theta = stringRecvd.substring(kd_pos+1).toFloat();
  }

  String encoderSend = String(char(startMarker)) + "F" + String(phi_en) + "T" + String(theta_en) + "S" + String(psi_en) + String(char(endMarker));
  Serial.write(encoderSend.c_str());
}
