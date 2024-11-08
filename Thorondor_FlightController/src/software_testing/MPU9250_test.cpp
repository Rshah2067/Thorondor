#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"
#include "madgwick9dof.cpp"

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

float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;


void read_IMU();
void general_calibration();


void setup() {
    // serial to display data
    Serial.begin(115200);

    // only uncomment to calibrate magnetometer
    general_calibration();
}

void loop() {
    // read the sensor
    read_IMU();

    // display the data
    /*
    Serial.print("Accel XYZ: ");
    Serial.print(Ax);
    Serial.print(", ");
    Serial.print(Ay);
    Serial.print(", ");
    Serial.println(Az);

    Serial.print("Gyro XYZ: ");
    Serial.print(Gx);
    Serial.print(", ");
    Serial.print(Gy);
    Serial.print(", ");
    Serial.println(Gz);

    Serial.print("Mag XYZ: ");
    Serial.print(Mx);
    Serial.print(", ");
    Serial.print(My);
    Serial.print(", ");
    Serial.println(Mz);
    delay(20);
    */

    Madgwick(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz, 200);

    Serial.println("Filtered Orientation");
    Serial.print(pitch_IMU);
    Serial.print(", ");
    Serial.print(yaw_IMU);
    Serial.print(", ");
    Serial.println(roll_IMU);

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