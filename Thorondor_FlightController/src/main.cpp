#include <Arduino.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// Initializes sensor as an object and assigns it unique ID
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Ada
void display_sensor_details();

void setup() {
  Serial.begin(115200);
  while(!Serial){

  }
  Serial.println("Serial inited");
  // Throws error if LSM303 is not detected
  if(!accel.begin()) {
    Serial.println("No LSM303 detected. Check wiring.");
    while(1);
  }

  display_sensor_details();
  
  // Sets the accelerometer range to 4G
  accel.setRange(LSM303_RANGE_4G);

  // Sets accelerometer to high resolution, note: draws more power
  accel.setMode(LSM303_MODE_HIGH_RESOLUTION);

}

void loop() {

  sensors_event_t event;
  accel.getEvent(&event);
  
  delay(250);
}
