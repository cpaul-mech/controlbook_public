void pythonCommunicate(float phi, float theta, float psi, float &theta_ref, float &km, float &kp, float &ki, float &kd) {
  if (Serial.available()) {
    // There's data incoming
    byte x = Serial.read();
    if (x == startMarker) {
      bytesRecvd = 0;
      inProgress = true;
    }

    if(inProgress) {
      tempBuffer[bytesRecvd] = x;
      bytesRecvd ++;
    }

    if (x == endMarker) {
      inProgress = false;
      String stringRecvd = "";
      for (byte n=1; n < bytesRecvd - 1 ; n++) {
        stringRecvd += (char)tempBuffer[n];
      }

      int tR_pos = stringRecvd.indexOf('R');
      int km_pos = stringRecvd.indexOf('K');
      int kp_pos = stringRecvd.indexOf('P');
      int ki_pos = stringRecvd.indexOf('I');
      int kd_pos = stringRecvd.indexOf('D');

      if (tR_pos != -1 && km_pos != -1 && kp_pos != -1 && ki_pos != -1 && kd_pos != -1) {
        String thetaRefValueStr = stringRecvd.substring(tR_pos+1, km_pos);
        String KmValueStr = stringRecvd.substring(km_pos+1, kp_pos);
        String KpValueStr = stringRecvd.substring(kp_pos+1, ki_pos);
        String KiValueStr = stringRecvd.substring(ki_pos+1, kd_pos);
        String KdValueStr = stringRecvd.substring(kd_pos+1);

        theta_ref = thetaRefValueStr.toFloat();
        km = KmValueStr.toFloat();
        kp = KpValueStr.toFloat();
        ki = KiValueStr.toFloat();
        kd = KdValueStr.toFloat();
      }
      String encoderSend = String(char(startMarker)) + "F" + String(phi) + "T" + String(theta) + "S" + String(psi) + String(char(endMarker));
      Serial.write(encoderSend.c_str());
    }
  }
}


