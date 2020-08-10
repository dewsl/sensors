#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);



void setup(void) {
  Serial.begin(9600);

  /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }
  
  /* Initialise the sensor */
  if (!gyro.begin()) {
    /* There was a problem detecting the FXAS21002C ... check your connections
     */
    //Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1)
      ;
  }
  if (!accelmag.begin(ACCEL_RANGE_4G)) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    //Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }

}

void loop(void) {
  sensor_get_data();
  
  delay(5000);
}
