#include "parsing.h"

String get_serial() {
    if (Serial.available() > 0) {

        String input_string = Serial.readStringUntil('\n');

        //Serial.print("Raw String: ");
        //Serial.println(input_string);

        return input_string;
    }
    return "";
}

int* parse_serial(String input) {
    /*
    Input must be 4 integers with comma delimeters!!!
    Value representations in order of input: left motor, right motor, left servo, right servo
    */
    static int actuation_vals[4];
    
    int count = 0; // Counter for how many values we have parsed
    int start = 0;
    int end = input.indexOf(",");

    while (end != -1 && count < 4) { // Continue while there are delimiters and space in array
        String temp = input.substring(start, end);
        actuation_vals[count] = temp.toInt(); // Store the parsed value
        count++; // Increment the count
        start = end + 1; // Move start to the next character
        end = input.indexOf(",", start); // Find the next delimiter
    }

    // Capture the last value (after the last comma)
    if (start < input.length() && count < 4) {
        actuation_vals[count] = input.substring(start).toInt();
        count++; // Increment the count for the last value
    }


    // Print the parsed values
    for (int i = 0; i < count; i++) {
        Serial.print(actuation_vals[i]);
        if (i < count - 1) {
            Serial.print(", ");
        }
    }
    Serial.println("");
    

    return actuation_vals;
}
