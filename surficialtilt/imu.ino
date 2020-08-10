void sensor_get_data(){
  /* Get a new sensor event */
  sensors_event_t event,aevent, mevent;

  gyro.getEvent(&event);
  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);
  int temp = 0;

    /* Display the accel results (acceleration is measured in m/s^2) */
//  Serial.print("A ");
//  Serial.print("X: ");
  Serial.print(aevent.acceleration.x, 4);
  Serial.print(",");
//  Serial.print("Y: ");
  Serial.print(aevent.acceleration.y, 4);
  Serial.print(",");
//  Serial.print("Z: ");
  Serial.print(aevent.acceleration.z, 4);
  Serial.print(",");
//  Serial.println("m/s^2");

  /* Display the mag results (mag data is in uTesla) */
//  Serial.print("M ");
//  Serial.print("X: ");
  Serial.print(mevent.magnetic.x, 1);
  Serial.print(",");
//  Serial.print("Y: ");
  Serial.print(mevent.magnetic.y, 1);
  Serial.print(",");
//  Serial.print("Z: ");
  Serial.print(mevent.magnetic.z, 1);
  Serial.print(",");
//  Serial.println("uT");

  /* Display the results (speed is measured in rad/s) */
//  Serial.print("X: ");
  Serial.print(event.gyro.x);
  Serial.print(",");
//  Serial.print("Y: ");
  Serial.print(event.gyro.y);
  Serial.print(",");
//  Serial.print("Z: ");
  Serial.print(event.gyro.z);
  Serial.print(",");
  Serial.print(temp);
  
  Serial.println("");  
}
