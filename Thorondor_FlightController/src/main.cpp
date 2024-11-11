#include<Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include<Servo.h>
#include<madwick9dof.h>
#include<MPU9250.h>
//Initialize Servos and Motors
Servo starboardMotor;
Servo starboardServo;
Servo portMotor;
Servo portServo;
//Initialize IMU
MPU9250 mpu9250(Wire,0x68);
//IMU Error Parameters
const float MagErrorX = 0.0f;
const float MagErrorY = 0.0f;
const float MagErrorZ = 0.0f;
const float MagScaleX = 1.0f;
const float MagScaleY = 1.0f;
const float MagScaleZ = 1.0f;
//Define the 
float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;

//NOTES:
//Starboard servo nuetral position is 20, max back is 10
void setup() {
  //Start Serial
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
void IMU_init() {
    int status = mpu9250.begin();

    if (status < 0) {
      Serial.println("MPU9250 initialization unsuccessful, check wiring.");
      while(1);
    }

    // set desired fullscale ranges for the sensors
    mpu9250.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    mpu9250.setAccelRange(MPU9250::ACCEL_RANGE_4G);

    // setting calibration parameters for magnetometer
    mpu9250.setMagCalX(MagErrorX, MagScaleX);
    mpu9250.setMagCalY(MagErrorY, MagScaleY);
    mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
    mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
}

void read_IMU() {
    // Reads sensor data and applies them to variables
    mpu9250.readSensor();

    Ax = mpu9250.getAccelX_mss();
    Ay = mpu9250.getAccelY_mss();
    Az = mpu9250.getAccelZ_mss();

    Gx = mpu9250.getGyroX_rads();
    Gy = mpu9250.getGyroY_rads();
    Gz = mpu9250.getGyroZ_rads();

    Mx = mpu9250.getMagX_uT();
    My = mpu9250.getMagY_uT();
    Mz = mpu9250.getMagZ_uT();
}

void general_calibration() {
    // Calibration with prompts for all 3 sensors in the IMU
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    delay(5000);
    mpu9250.calibrateAccel();
    mpu9250.calibrateGyro();
    
    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu9250.calibrateMag();

    // Printing calibration result parameters
    Serial.println("Calibration Parameters:");

    Serial.println("Accelerometer Bias XYZ:");
    Serial.print(mpu9250.getAccelBiasX_mss());
    Serial.print(",");
    Serial.print(mpu9250.getAccelBiasY_mss());
    Serial.print(",");
    Serial.println(mpu9250.getAccelBiasZ_mss());

    Serial.println("Gyro Bias XYZ:");
    Serial.print(mpu9250.getGyroBiasX_rads());
    Serial.print(",");
    Serial.print(mpu9250.getGyroBiasY_rads());
    Serial.print(",");
    Serial.print(mpu9250.getGyroBiasZ_rads());

    Serial.println("Magnetometer Bias XYZ:");
    Serial.print(mpu9250.getMagBiasX_uT());
    Serial.print(",");
    Serial.print(mpu9250.getMagBiasY_uT());
    Serial.print(",");
    Serial.print(mpu9250.getMagBiasZ_uT());
}