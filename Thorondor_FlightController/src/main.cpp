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
// Servo myservo;  // create servo object to control a servo
// // twelve servo objects can be created on most boards


// void setup() {
//   Serial.begin(11520);
//   myservo.attach(2,1000,2000);// attaches the servo on GIO2 to the servo object
//   myservo.write(0);
//   Serial.println("Waiting for Startup");
//   delay(3000);
  
  
// }

// void loop() {
//   Serial.println("Running");
//   if(digitalRead(16)){
//       myservo.write(50);
//   }
//   else{
//     myservo.write(0);
//   }
// }