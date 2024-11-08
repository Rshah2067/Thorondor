/*
#include <Arduino.h>
#include "parsing.h"

String input;
int motor_L;
int motor_R;
int servo_L;
int servo_R;

void setup() {
    Serial.begin(9600);
    int actuation_vals[4];
}

void loop() {
    input = get_serial();
    if (input.length() > 0) {
        int* actuation_vals = parse_serial(input);

        motor_L = actuation_vals[0];
        motor_R = actuation_vals[1];
        servo_L = actuation_vals[2];
        servo_R = actuation_vals[3];

        Serial.println(motor_L);
        Serial.println(motor_R);
        Serial.println(servo_L);
        Serial.println(servo_R);
    }
}
*/