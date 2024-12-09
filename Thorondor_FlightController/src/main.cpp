#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include "MPU9250.h"

/*
Several directive choices
IMU_CALIBRATION: Only triggers IMU calibration
IMU_TEST: Triggers IMU testing
YAW_TEST: Allows yaw testing
*/
#define NONE

/*
OWN_FUNC: using own IMU udpate functions
LIB_FUNC: using library IMU update functions
*/
#define OWN_FUNC


//variables for reading Radio Signal
unsigned long int a, b, c;
//specifing arrays and variables to store values 
int x[15], ch1[15], ch[9], i;


//Initialize Servos and Motors
Servo starboardMotor;
Servo starboardServo;
Servo portMotor;
Servo portServo;

//Initialize IMU
MPU9250 mpu;

// IMU error parameters, manually enter them after calibration
// These values are calibrated for test stand purposes
const float MagErrorX = 343.54f;
const float MagErrorY = -354.35f;
const float MagErrorZ = 814.17f;
const float MagScaleX = 1.59f;
const float MagScaleY = 1.12f;
const float MagScaleZ = 0.68f;

const float AccErrorX = 1.89f;
const float AccErrorY = -27.41f;
const float AccErrorZ = 2.87f;

const float GyroErrorX = 8.59f;
const float GyroErrorY = -2.53f;
const float GyroErrorZ = -0.62f;

// Defining IMU variables
float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;

// Madgwick filter gradient descent iterations
const int n_filter_iter = 10;

// Defining pitch, roll, yaw angles
float pitch_angle;
float roll_angle;
float yaw_angle;

// Defining desired controller values
float desired_throttle;
float desired_roll;
// Defining a vector to hold quaternion
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// Setting Madgwick Filter parameters
float GyroMeasError = PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float B_madgwick = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta value
float dT = 0.005; // Setting timestep for Madgwick (seconds)

// Setting PID Constants
float PIDtimer;
float pitch_error;
float roll_error;
float Rp = 0.005f;
float Ri = 0.0f;
float Rd = 0.01f;
float Pp = 0.75f;
float Pi = 0.0f;
float Pd = 0.25f;

// Declaring functions
float pitch_PID(float angle,float setpoint,float P,float I, float D);
float roll_PID(float angle,float setpoint,float P,float I, float D);
void IMU_init();
void read_IMU();
void update_state();
void print_calibration();
float invSqrt(float x);
void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
float degree2ms(float degrees);
void read_me();
void read_rc();

//NOTES:
//Starboard servo nuetral position is 135, max back is 180
//Port servo nuetral is 75 max back is 0
//Servo Gear ratio is 2:1
void setup() {
  //Start Serial
  Serial.begin(115200);
  
  
  pinMode(28, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(28), read_me, FALLING);
  while(!Serial){
    
  }
  delay(5000);

  
  Serial.println("Type 'go' to start.");
  Wire.begin();
  // //wait for the radio to connect
  // while (ch[1] ==-1000 && ch[2] == -1000 && ch[3] == -1000 && ch[4] ==-1000 && ch[5]==-1000){
  //   read_rc();
  //   Serial.println("Waiting For Connection");
  // }
  // Serial.print(ch[1]);Serial.print("\t");
  // Serial.print(ch[2]);Serial.print("\t");
  // Serial.print(ch[3]);Serial.print("\t");
  // Serial.print(ch[4]);Serial.print("\t");
  // Serial.print(ch[5]);Serial.print("\t");
  // Serial.print(ch[6]);Serial.print("\n");
  portMotor.attach(2,1000,2000);// attaches the servo on GIO2 to the servo object
  starboardMotor.attach(3,1000,2000);
  starboardMotor.write(0);
  portMotor.write(0);
  portServo.attach(9);
  portServo.write(70);
  starboardServo.attach(7);
  starboardServo.write(57);
  IMU_init();
  delay(500);
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

  // Initializing PID interval timer
  PIDtimer = millis();

  // Enables IMU calibration functionality, stops in an infinite loop
  #ifdef IMU_CALIBRATION
    // IMU Calibration
    // Store all necessary calibration parameters into corresponding variables
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);

    while(1);
  #endif
  
}

void loop() {
  // Main flight controller loop

  Serial.print("");

  //read radio commands
  read_rc();
  /*
  Serial.print(ch[1]);Serial.print("\t");
  Serial.print(ch[2]);Serial.print("\t");
  Serial.print(ch[3]);Serial.print("\t");
  Serial.print(ch[4]);Serial.print("\t");
  Serial.print(ch[5]);Serial.print("\t");
  Serial.print(ch[6]);Serial.print("\n");
  */

  // Get IMU data and process it to update orientation
  read_IMU();
  update_state();

  // 0.18 converts PPM (Range 0-1000) to 0-180 for Servo.write()
  desired_throttle = ch[3] * 0.18;
  
  // desired_roll = (ch[1]-500)*.05;
  // Serial.println(desired_throttle);
  // starboardMotor.write(desired_throttle-desired_roll);
  // portMotor.write(desired_throttle+desired_roll);
  

  //Start PID cycle timer
  if (millis() - PIDtimer > 50){
    // Calculating pitch correction
    float pitch_correction = pitch_PID(pitch_angle, 0, Pp, Pd, Pi);
    if (portServo.read() - pitch_correction <= 70.0f - 35.0f) {
      portServo.writeMicroseconds(degree2ms(70.0f - 35.0f));
    } else if (portServo.read() + pitch_correction >= 70.0f + 35.0f) {
      portServo.writeMicroseconds(degree2ms(70.0f + 35.0f));
    } else {
      portServo.writeMicroseconds(degree2ms(70.0f - pitch_correction));
    }
    if (starboardServo.read() - pitch_correction <= 57.0f - 35.0f) {
      starboardServo.writeMicroseconds(degree2ms(57.0f - 35.0f));
    } else if (starboardServo.read() + pitch_correction >= 57.0f + 35.0f) {
      starboardServo.writeMicroseconds(degree2ms(57.0f + 35.0f));
    } else {
      starboardServo.writeMicroseconds(degree2ms(57.0f - pitch_correction));
    }

    /*
    Serial.print(pitch_angle);
    Serial.print(" Port ");
    Serial.print(portServo.read());
    Serial.print(" Starboard ");
    Serial.print(starboardServo.read());
    Serial.print(" Correction ");
    Serial.println(pitch_correction);
    */

    // Calculating roll correction
    float roll_correction = roll_PID(roll_angle, 0, Rp, Ri, Rd);
    // Prevent overflow if write values are too large or small
    if (starboardMotor.read() - roll_correction >= 180){
      starboardMotor.writeMicroseconds(degree2ms(180));
    }
    else if (starboardMotor.read() - roll_correction <=0){
      starboardMotor.writeMicroseconds(degree2ms(0));
    }
    else{
      starboardMotor.writeMicroseconds(degree2ms(desired_throttle - roll_correction));
    }
    if (portMotor.read() + roll_correction >= 180){
      portMotor.writeMicroseconds(degree2ms(180));
    }
    else if (portMotor.read() + roll_correction <= 0){
      portMotor.writeMicroseconds(degree2ms(0));
    }
    else{
      portMotor.writeMicroseconds(degree2ms(desired_throttle + roll_correction));
    }
    PIDtimer = millis();
    
    
    Serial.print(desired_throttle);
    Serial.print(" ");
    Serial.print(roll_angle);
    Serial.print(" Port ");
    Serial.print(portMotor.read());
    Serial.print(" Starboard ");
    Serial.print(starboardMotor.read());
    Serial.print(" Correction ");
    Serial.println(roll_correction);
    
  }
}
    

// When there is positive error, aircraft is rolled to port of desired angle (positive correction should be added to the port motor, substracted form starboard)
// When there is positive error, aircraft is rolled to starboard of desired angle (positive correction should be added to starboard motor, substracted form port)
float roll_PID(float angle, float setpoint, float P, float I, float D){
  // Calculates correction signal for roll control
  float previous_roll_error = roll_error;
  roll_error = setpoint - angle;
  float correction = P * roll_error + I * roll_error * 0.05f + D * (roll_error - previous_roll_error) / 0.05f;
  return correction;
}

// When there is positive error, aircraft is pitched forward of desired angle (servos need to rotate backwards)
// When there is negative error, aircraft is pitched backwards of desired angle (servo need to rotate forwards)
float pitch_PID(float angle, float setpoint, float P, float I, float D){
  // Calculates correction signal for pitch control
  float previous_pitch_error = pitch_error;
  pitch_error = setpoint - angle;
  float correction = P * pitch_error + I * pitch_error * 0.05f + D * (roll_error - previous_pitch_error) / 0.05f;
  return correction;
}


void IMU_init() {
  /* 
  Various IMU Settings:
  - Acc range is 4Gs
  - Gyro range is 500 DPS
  - Mag output resolution is 16 bits
  - Sampling rate is 200Hz
  - Configures low pass filter of 41Hz for gyro
  - Configures low pass filter of 45Hz for acc
  */
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  // Sets up IMU to default I2C register
  mpu.setup(0x68, setting);

  // If setup goes wrong, throw error and enter infinite loop
  if (!mpu.setup(0x68)) {
      while(1) {
        Serial.println("MPU Connection Failed. Check wiring.");
      }
  }

  // Set sensor bias and scaling to manually set values based off of calibration
  mpu.setAccBias(AccErrorX, AccErrorY, AccErrorZ);
  mpu.setGyroBias(GyroErrorX, GyroErrorY, GyroErrorZ);
  mpu.setMagBias(MagErrorX, MagErrorY, MagErrorZ);
  mpu.setMagScale(MagScaleX, MagScaleY, MagScaleZ);
    
  #ifdef LIB_FUNC
    // Selecting to use madgwick filter with 10 iterations for convergence
    // Tentative depending on independent implementation of madgwick
    mpu.selectFilter(QuatFilterSel::MADGWICK);
    mpu.setFilterIterations(10);
  #endif
}

void read_IMU() {
  
  #ifdef OWN_FUNC
    // This function is to update IMU values and assign them to corresponding variables
    // Functions that grab values already account for bias and scaling

    // update and read IMU values
    mpu.update_accel_gyro();
    mpu.update_mag();

    // functions for getting accelerometer and gyro data does not compensate for bias automatically
    Ax = mpu.getAccX();
    Ay = mpu.getAccY();
    Az = mpu.getAccZ();
    Gx = mpu.getGyroX();
    Gy = mpu.getGyroY();
    Gz = mpu.getGyroZ();

    // functions getting magnetometer data automatically scales and compensates for bias
    Mx = mpu.getMagX();
    My = mpu.getMagY();
    Mz = mpu.getMagZ();
  #endif

  #ifdef LIB_FUNC
    // If IMU updated, grab yaw, pitch, roll angles
    if(mpu.update()) {
      yaw_angle = mpu.getYaw();
      pitch_angle = mpu.getPitch();
      roll_angle = mpu.getRoll();
    }
  #endif
}

void update_state() {
  // This function overall applies filtering to processed IMU data and then converts it into aircraft reference frame angles

  // Madgwick filter needs to be fed North, East, and Down direction like
  // (AN, AE, AD, GN, GE, GD, MN, ME, MD)
  // Accel and Gyro reference frame uses Right Hand Rule, where X-Forward and Z-Up
  // Mag reference frame uses Right Hand Rule, where Y-Forward and Z-Down
  // Transform Accel, Gyro, and Mag reference frame into general Aircraft coordinate system, which uses Right Hand Rule, where X-Forward and Z-Down
  // Madgwick inputs should be (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
  // but instead, we input (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
  // because gravity is by convention positive down, we need to ivnert the accel data

  // get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
  // acc (mg_, gyro (deg/s), mag (mG)
  // gyro will be convert from (deg/s) to (rad/s) inside of this function
  
  // defining North, East, Down directions for corresponding sensor values
  float AN = -Ax;
  float AE = Ay;
  float AD = Az;

  float GN = Gx * DEG_TO_RAD;
  float GE = -Gy * DEG_TO_RAD;
  float GD = -Gz * DEG_TO_RAD;

  float MN = My;
  float ME = -Mx;
  float MD = Mz;

  // 10 iterations of filtering for solution convergence
  for (int i = 0; i < 10; i++) {
    madgwick(AN, AE, AD, GN, GE, GD, MN, ME, MD, q);
  }

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  
  // Assigning quaternion values for matrix operations
  float qw = q[0];
  float qx = q[1];
  float qy = q[2];
  float qz = q[3];

  // rotation matrix coefficients for Euler angles and gravity components
  float a12, a22, a31, a32, a33;  
  a12 = 2.0f * (qx * qy + qw * qz);
  a22 = qw * qw + qx * qx - qy * qy - qz * qz;
  a31 = 2.0f * (qw * qx + qy * qz);
  a32 = 2.0f * (qx * qz - qw * qy);
  a33 = qw * qw - qx * qx - qy * qy + qz * qz;

  // converting to roll, pitch, yaw
  roll_angle = atan2f(a31, a33) * RAD_TO_DEG;
  pitch_angle = -asinf(a32) * RAD_TO_DEG;
  yaw_angle = atan2f(a12, a22) * RAD_TO_DEG;

  // If yaw angle exceeds 180, make it negative, vice versa
  if (yaw_angle >= +180.f) {
    yaw_angle -= 360.f;
  } else if (yaw_angle < -180.f) {
    yaw_angle += 360.f; 
  }

}

void print_calibration() {
  // Print calibration data
  Serial.println("Calibration Parameters:");
  Serial.println("Accelerometer Bias (g): ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("Gyro Bias (deg/s): ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("Magnetometer Bias (mG): ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("Magnetometer Scale: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();
}

float invSqrt(float x) {
  // Fast inverse sqrt algorithm

  // Implementation of the famous "fast inverse square root" algorithm, 
  // which provides an approximation for calculating 1/sqrt(x) more efficiently 
  // than the standard method. The algorithm uses a bit manipulation trick followed 
  // by two iterations of Newton's method to refine the approximation.

  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y; // trick computer into thinking it's using an integer (EVIL BLACK MAGIC!!!!!!)
  i = 0x5f3759df - (i>>1); // black magic with bit shifting
  y = *(float*)&i; // reverses int approximation back into float
  y = y * (1.5f - (halfx * y * y)); // 2 iterations of Newton's method to improve solution
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
  // Madgwick filter used for orientation estimation

  // This function implements the Madgwick filter algorithm, which is used to 
  // estimate the orientation of a sensor in 3D space by combining accelerometer, 
  // gyroscope, and magnetometer data. The filter uses a gradient descent algorithm 
  // to minimize the error between the estimated orientation and the sensor's readings, 
  // providing real-time quaternion values for accurate orientation estimation.

  // The filter computes the quaternion values (q0, q1, q2, q3) representing the 
  // orientation, and updates the sensor's orientation estimate based on the 
  // input data from the sensors.

  // Declaring relevant quaternion and quaternion rate variables
  double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3]; // splitting quaternion up into components
  double recipNorm; // inverse square root for normalizing quaternions
  double s0, s1, s2, s3; // sensor frame vector components
  double qDot1, qDot2, qDot3, qDot4; // gyro rate represented as quaternion
  double hx, hy; // Magnetometer components
  
  // Variables to prevent repeated arithmetic
  double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Normalise accelerometer measurement
  float a_norm = ax * ax + ay * ay + az * az;
  if (a_norm == 0.) return;  // handle NaN
  recipNorm = invSqrt(a_norm);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // Normalise magnetometer measurement
  float m_norm = mx * mx + my * my + mz * mz;
  if (m_norm == 0.) return;  // handle NaN
  recipNorm = invSqrt(m_norm);
  mx *= recipNorm;
  my *= recipNorm;
  mz *= recipNorm;

  // Auxiliary variables to avoid repeated arithmetic
  _2q0mx = 2.0f * q0 * mx;
  _2q0my = 2.0f * q0 * my;
  _2q0mz = 2.0f * q0 * mz;
  _2q1mx = 2.0f * q1 * mx;
  _2q0 = 2.0f * q0;
  _2q1 = 2.0f * q1;
  _2q2 = 2.0f * q2;
  _2q3 = 2.0f * q3;
  _2q0q2 = 2.0f * q0 * q2;
  _2q2q3 = 2.0f * q2 * q3;
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  // Reference direction of Earth's magnetic field
  // "Somewhat black magic"
  hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
  hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  // "I have no clue what's going on, help ;-;""
  s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
  s0 *= recipNorm;
  s1 *= recipNorm;
  s2 *= recipNorm;
  s3 *= recipNorm;

  // Apply feedback step
  // Complimentary filtering between grad descent and gyro results
  qDot1 -= B_madgwick * s0;
  qDot2 -= B_madgwick * s1;
  qDot3 -= B_madgwick * s2;
  qDot4 -= B_madgwick * s3;

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dT;
  q1 += qDot2 * dT;
  q2 += qDot3 * dT;
  q3 += qDot4 * dT;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Updating components
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

float degree2ms(float degrees) {
  // converts degrees to microseconds for Servo.writeMicroseconds()
  return 1000.0 + degrees * 50.0/9.0;
}

void read_me() {
    // This code reads values from an RC receiver from the PPM pin (Pin 2 or 3)
    // It provides channel values from 0 to 1000
    //    -: ABHILASH :-    //

    a = micros();  // Store the time value 'a' when the pin value falls
    c = a - b;     // Calculate the time between two peaks
    b = a; 
    x[i] = c;      // Store the time difference in the temporary array 'x'
    i = i + 1;     // Increment the index 'i'

    if (i == 15) {
        // After 15 readings, copy the values from the temporary array 'x' to 'ch1'
        for (int j = 0; j < 15; j++) {
            ch1[j] = x[j];  
        }
        i = 0;  // Reset the index 'i'
    }
}

void read_rc() {
    int i, j, k = 0;

    // Loop to find the separation space (10000us) and mark the index 'j'
    for (k = 14; k > -1; k--) {
        if (ch1[k] > 5000) {
            j = k;  // Mark the index where the separation occurs
        }
    }

    // Assign values to channels (after separation space) and adjust the values
    for (i = 1; i <= 8; i++) {
        ch[i] = (ch1[i + j] - 1000);  // Subtract 1000 to scale the values
    }
}
