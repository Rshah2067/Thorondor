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
#define PITCH_TEST

/*
OWN_FUNC: using own IMU udpate functions
LIB_FUNC: using library IMU update functions
*/
#define LIB_FUNC

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

// Defining a vector to hold quaternion
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  

// Setting timestep for Madgwick
float dT = 0.005;

// Setting beta value based off of these parameters
float GyroMeasError = PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float B_madgwick = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
//Setting PID Constants
float PIDtimer;
float error;
float Rp = .225;
float Ri = 0.15f;
float Rd = 0.15f;
float Pp = 2.0f;
float Pi = 0.0f;
float Pd = 4.0f;
float PID(float angle,float setpoint,float P,float I, float D);
void IMU_init();
void read_IMU();
void print_calibration();
float invSqrt(float x);
void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void update_state();

//NOTES:
//Starboard servo nuetral position is 135, max back is 180
//Port servo nuetral is 75 max back is 0
//Servo Gear ratio is 2:1
void setup() {
  //Start Serial
  Serial.begin(115200);
  Wire.begin();
  delay(5000);
  
  // Enables yaw test functionality
  while(!Serial){
    
  }
  portMotor.attach(2,1000,2000);// attaches the servo on GIO2 to the servo object
  starboardMotor.attach(3,1000,2000);
  starboardMotor.write(0);
  portMotor.write(0);
  portServo.attach(9);
  portServo.write(75);
  starboardServo.attach(7);
  starboardServo.write(115);
  
  IMU_init();
  delay(300);
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

  // Setting base speed value to motors
  portMotor.write(40);
  starboardMotor.write(40);

  // Initializing IMU
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
  //Read from Serial to see if we have a new PID constant
  // Reads IMU data and as signs them to corresponding variables
  // if (Serial.available() > 0) {
  //   String input = Serial.readStringUntil('\n');  // Read the serial input until newline character

  //   // Ensure input is long enough to have a letter followed by a number
  //   if (input.length() > 1) {
  //     char id = input.charAt(0);  // Get the first character (P, I, or D)
  //     float value = input.substring(1).toFloat(); // Convert the remaining part to a float

  //     switch (id) {
  //       case 'P':
  //         P = value;
  //         Serial.print("Kp updated to: ");
  //         Serial.println(P);
  //         break;

  //       case 'I':
  //         I = value;
  //         Serial.print("Ki updated to: ");
  //         Serial.println(I);
  //         break;

  //       case 'D':
  //         D = value;
  //         Serial.print("Kd updated to: ");
  //         Serial.println(D);
  //         break;

  //       default:
  //         Serial.println("Invalid input. Use P, I, or D followed by a number.");
  //         break;
  //     }
  //   } else {
  //     Serial.println("Invalid input format. Use P, I, or D followed by a number.");
  //   }
  // }
  read_IMU();
  //Start cycle timer

  if (millis() - PIDtimer > 50){
      // a positive pitch corresponds to the aircraft pitching "forward"
      //that means when we get a positive correction we want to rotors to tilt back
      float pitch_correction = PID(pitch_angle,0,Pp,Pd,Pi);
      portServo.write(75+pitch_correction);
      starboardServo.write(115-pitch_correction); 
      Serial.print(pitch_angle);
      Serial.print(" Port ");
      Serial.print(portServo.read());
      Serial.print(" Starboard ");
      Serial.print(starboardServo.read());
      Serial.print(" Correction ");
      Serial.println(pitch_correction);
  }
  //Run PID and apply corrective motor powers
  //   float roll_correction = PID(roll_angle,0,RP,RI,RD);
  //   //prevent overflow
  //   if (starboardMotor.read()-roll_correction >= 180){
  //     starboardMotor.write(180);
  //   }
  //   else if (starboardMotor.read()-roll_correction <=0){
  //     starboardMotor.write(0);
  //   }
  //   else{
  //     starboardMotor.write(40.0f-roll_correction);
  //   }
  //   if (portMotor.read()+roll_correction >= 180){
  //     portMotor.write(180);
  //   }
  //   else if (portMotor.read()+roll_correction <= 0){
  //     portMotor.write(0);
  //   }
  //   else{
  //     portMotor.write(40.0f+roll_correction);
  //   }
  //   PIDtimer = millis();
  //   Serial.print(roll_angle);
  //   Serial.print(" Port ");
  //   Serial.print(portMotor.read());
  //   Serial.print(" Starboard ");
  //   Serial.print(starboardMotor.read());
  //   Serial.print(" Correction ");
  //   Serial.println(roll_correction);
  // }
  
}
//When I have negative Error that means we are rolled to port (port motor needs more power)
//When I have positive error that means we are rolled to starboard (starboard motor needs more power) (positive correction should be added to starboard, substracted form port)
float PID(float angle,float setpoint,float P, float I, float D){
  float previousError = error;
  error = setpoint-angle;
  float correction = P * error + I * error * 0.05f + D * (error-previousError) / 0.05f;
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

  // Set sensor bias and scaling accordingly
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
  // use either fast inverse sqrt or just regular computation, depending on valuing speed or accuracy
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
  // Basically like complimentary filtering
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
