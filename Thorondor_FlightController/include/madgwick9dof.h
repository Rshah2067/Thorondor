#include <Arduino.h>

float pitch_angle;
float roll_angle;
float yaw_angle;

float invSqrt(float x);
void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
