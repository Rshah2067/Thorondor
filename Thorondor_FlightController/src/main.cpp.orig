#include<Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include<Servo.h>
Servo starboardMotor;
Servo starboardServo;
Servo portMotor;
Servo portServo;
//NOTES:
//Starboard servo nuetral position is 20, max back is 10
void setup() {
  Serial.begin(9600);
  while(!Serial){
    
  }
  starboardMotor.attach(2,1000,2000);// attaches the servo on GIO2 to the servo object
  portMotor.attach(3,1000,2000);
  starboardMotor.write(0);
  portMotor.write(0);
  starboardServo.attach(4);
  starboardServo.write(20);
  portServo.attach(5);
  portServo.write(160);
  Serial.println("Type 'go' to start.");
  
  while (true) {
    if (Serial.available() > 0) {        // Check if data is available to read
      String input = Serial.readString(); // Read the input as a string
      
      input.trim();                       // Remove any leading/trailing whitespace
      
      if (input.equalsIgnoreCase("go")) { // Check if the input matches "go" (case-insensitive)
        Serial.println("Starting program...");
        break;                            // Exit the loop and continue the program
      } else {
        Serial.println("Invalid input. Type 'go' to start."); // Prompt again
      }
    }
  }
  //portServo.write(135);
  delay(300);
}

void loop() {
  Serial.println("running");
  starboardServo.write(50);
  portServo.write(170);
  portMotor.write(50);
  starboardMotor.write(50);
}
//Servo Gear ratio is 2:1