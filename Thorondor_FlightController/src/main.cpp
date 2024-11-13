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
#define NA 

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

// Initialize quaternion for madgwick filter
float q0 = 1.0f; 
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

// Setting beta value based off of these parameters
float GyroMeasError = PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float B_madgwick = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
//Setting PID Constants
//P at which the controller just starts to oscillate
// TODO ADD SOME I

float P = 0.148;
float I = 0.0f;
float D = 0.0f;
float PID(float roll,float setpoint);
void IMU_init();
void read_IMU();
void print_calibration();
float invSqrt(float x);
void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);

//NOTES:
//Starboard servo nuetral position is 20, max back is 10
//Servo Gear ratio is 2:1
void setup() {
  //Start Serial
  Serial.begin(115200);
  Wire.begin();
  delay(5000);
  
// Enables yaw test functionality
  while(!Serial){
    
  }
  starboardMotor.attach(2,1000,2000);// attaches the servo on GIO2 to the servo object
  portMotor.attach(3,1000,2000);
  starboardMotor.write(0);
  portMotor.write(0);
  starboardServo.attach(6);
  starboardServo.write(20);
  portServo.attach(7);
  portServo.write(160);
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
  portMotor.write(25);
  starboardMotor.write(25);
  // Initializing IMU
  
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
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the serial input until newline character

    // Ensure input is long enough to have a letter followed by a number
    if (input.length() > 1) {
      char id = input.charAt(0);  // Get the first character (P, I, or D)
      float value = input.substring(1).toFloat(); // Convert the remaining part to a float

      switch (id) {
        case 'P':
          P = value;
          Serial.print("Kp updated to: ");
          Serial.println(P);
          break;

        case 'I':
          I = value;
          Serial.print("Ki updated to: ");
          Serial.println(I);
          break;

        case 'D':
          D = value;
          Serial.print("Kd updated to: ");
          Serial.println(D);
          break;

        default:
          Serial.println("Invalid input. Use P, I, or D followed by a number.");
          break;
      }
    } else {
      Serial.println("Invalid input format. Use P, I, or D followed by a number.");
    }
  }
  read_IMU();
  //Run PID and apply corrective motor powers
  float correction = PID(roll_angle,0);
  //prevent overflow
  if (starboardMotor.read()-correction >= 180){
    starboardMotor.write(180);
  }
  else if (starboardMotor.read()-correction <=0){
    starboardMotor.write(0);
  }
  else{
    starboardMotor.write(30.0f-correction);
  }
  if (portMotor.read()+correction >= 180){
    portMotor.write(180);
  }
  else if (portMotor.read()+correction <= 0){
    portMotor.write(0);
  }
  else{
    portMotor.write(30.0f+correction);
  }
  Serial.print(roll_angle);
  Serial.print(" Port ");
  Serial.print(portMotor.read());
  Serial.print(" Starboard ");
  Serial.print(starboardMotor.read());
  Serial.print(" Correction ");
  Serial.println(correction);

}
//When I have negative Error that means we are rolled to port (port motor needs more power)
//When I have positive error that means we are rolled to starboard (starboard motor needs more power) (positive correction should be added to starboard, substracted form port)
float PID(float roll,float setpoint){
  float error = setpoint-roll;
  float correction = P*error;
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
    
  // Selecting to use madgwick filter with 10 iterations for convergence
  // Tentative depending on independent implementation of madgwick
  
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(10);
  

  /*
  // setting calibration parameters for magnetometer
  mpu.setMagBias(MagErrorX, MagErrorY, MagErrorZ);
  mpu.setMagScale(MagScaleX, MagScaleY, MagScaleZ);
  
  mpu.setAccBias(AccErrorX, AccErrorY, AccErrorZ);
  
  mpu.setGyroBias(GyroErrorX, GyroErrorY, GyroErrorZ);
  */
}

void read_IMU() {
  /*
  // functions for getting accelerometer and gyro data does not compensate for bias automatically
  Ax = mpu.getAccX() - mpu.getAccBiasX();
  Ay = mpu.getAccY() - mpu.getAccBiasY();
  Az = mpu.getAccZ() - mpu.getAccBiasZ();
  Gx = mpu.getGyroX() - mpu.getGyroBiasX();
  Gy = mpu.getGyroY() - mpu.getGyroBiasY();
  Gz = mpu.getGyroZ() - mpu.getGyroBiasZ();

  // functions getting magnetometer data automatically scales and compensates for bias
  Mx = mpu.getMagX();
  My = mpu.getMagY();
  Mz = mpu.getMagZ();
  */
  
  // Reads sensor data and applies them to variables (not sure if this includes filtering/bias offsetting)
  if (mpu.update()) {
    #ifdef IMU_TEST
      Serial.print(mpu.getYaw()); Serial.print(",");
      Serial.print(mpu.getPitch()); Serial.print(",");
      Serial.println(mpu.getRoll());
    #endif

    // stores angles into corresponding variables
    yaw_angle = mpu.getYaw();
    pitch_angle = mpu.getPitch();
    roll_angle = mpu.getRoll();
  }
}

void print_calibration() {
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
    // use either fast inverse sqrt or just regular computation, depending on valuing speed or accuracy
    // Fast inverse sqrt algorithm
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y; // trick computer into thinking it's using an integer (EVIL!!!!!!)
    i = 0x5f3759df - (i>>1); // black magic with bit shifting
    y = *(float*)&i; // reverses int approximation back into float
    y = y * (1.5f - (halfx * y * y)); // 2 iterations of Newton's method to improve solution
    y = y * (1.5f - (halfx * y * y));
    return y;

    //return 1.0/sqrtf(x); // Teensy is fast enough to just take the compute penalty, but is a RP2040 as fast?
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
    //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
    /*
    * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
    * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
    * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
    * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
    * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
    */
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    //Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    //Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement for each component
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement for each component
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
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

        //Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        //Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        // normalise step magnitude for each component
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //Apply feedback step
        qDot1 -= B_madgwick * s0;
        qDot2 -= B_madgwick * s1;
        qDot3 -= B_madgwick * s2;
        qDot4 -= B_madgwick * s3;
    }

    //Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    //Normalize quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //compute angles - NWU
    roll_angle = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
    pitch_angle = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
    yaw_angle = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}
