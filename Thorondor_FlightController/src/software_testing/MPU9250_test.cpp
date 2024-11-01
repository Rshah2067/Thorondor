#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 mpu9250(Wire,0x68);

int status;

// Magnetometer error parameters
// ONLY CHANGE TO CORRESPONDING VALUES AFTER CALIBRATION
float MagErrorX = 0.0f;
float MagErrorY = 0.0f;
float MagErrorZ = 0.0f;
float MagScaleX = 1.0f;
float MagScaleY = 1.0f;
float MagScaleZ = 1.0f;



void setup() {
    // serial to display data
    Serial.begin(115200);

    // only uncomment to calibrate magnetometer
    calibrateMagnetometer();

}

void loop() {
    // read the sensor
    mpu9250.readSensor();

    // display the data
    Serial.print("Accel XYZ: ");
    Serial.print(mpu9250.getAccelX_mss(),6);
    Serial.print(", ");
    Serial.print(mpu9250.getAccelY_mss(),6);
    Serial.print(", ");
    Serial.println(mpu9250.getAccelZ_mss(),6);

    Serial.print("Gyro XYZ: ");
    Serial.print(mpu9250.getGyroX_rads(),6);
    Serial.print(", ");
    Serial.print(mpu9250.getGyroY_rads(),6);
    Serial.print(", ");
    Serial.println(mpu9250.getGyroZ_rads(),6);

    Serial.print("Mag XYZ: ");
    Serial.print(mpu9250.getMagX_uT(),6);
    Serial.print(", ");
    Serial.print(mpu9250.getMagY_uT(),6);
    Serial.print(", ");
    Serial.println(mpu9250.getMagZ_uT(),6);
    delay(20);
}

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

void calibrateMagnetometer() {
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = mpu9250.calibrateMag();
    if(success) {
        Serial.println("Calibration Successful!");
        Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
        Serial.print("float MagErrorX = ");
        Serial.print(mpu9250.getMagBiasX_uT());
        Serial.println(";");
        Serial.print("float MagErrorY = ");
        Serial.print(mpu9250.getMagBiasY_uT());
        Serial.println(";");
        Serial.print("float MagErrorZ = ");
        Serial.print(mpu9250.getMagBiasZ_uT());
        Serial.println(";");
        Serial.print("float MagScaleX = ");
        Serial.print(mpu9250.getMagScaleFactorX());
        Serial.println(";");
        Serial.print("float MagScaleY = ");
        Serial.print(mpu9250.getMagScaleFactorY());
        Serial.println(";");
        Serial.print("float MagScaleZ = ");
        Serial.print(mpu9250.getMagScaleFactorZ());
        Serial.println(";");
        Serial.println(" ");
        Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else {
        Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
    }
  
    while(1); //Halt code so it won't enter main loop until this function commented out
}