#include "ServoEx.h"
#include "Kinematics.h"

ServoEx ArmServo[5];

//=============================================================================
// Setup
//=============================================================================
void setup(){
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(5, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(7, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(9, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(11, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(13, GRIPPER_MIN, GRIPPER_MAX);

  // Initialize servo position
  doArmIK(0, 185, 185, 0);
  
  // Start serial
  Serial.begin(115200);
}

// Define number of pieces
const int numberOfPieces = 3;
String pieces[numberOfPieces];

// This will be the buffered string from Serial.read() up until \n
// Should look something like "123,456,789,"
String input = "";

// Keep track of current position in array
int counter = 0;

// Keep track of the last comma so we know where to start the substring
int lastIndex = 0;

void loop(){
  // Check for data coming in from serial
  if (Serial.available() > 0) {
    
    // Read the first byte and store it as a char
    char ch = Serial.read();
    
    // Do all the processing here since this is the end of a line
    if (ch == '\n' || ch == '\r') {
      for (int i = 0; i < input.length(); i++) {
        // Loop through each character and check if it's a comma
        if (input.substring(i, i+1) == ",") {
          // Grab piece from last index up to the current position and store it
          pieces[counter] = input.substring(lastIndex, i);
          // Update last position and add 1, so it starts from next character
          lastIndex = i + 1;
          // Increase the position in the array that we store into
          counter++;
        }

        // If we're at the end of the string (no more commas to stop us)
        if (i == input.length() - 1) {
          // Grab the last part of the string from the lastIndex to the end
          pieces[counter] = input.substring(lastIndex, i);
        }
      }

      // Update the position
      float x = pieces[0].toFloat();
      float y = pieces[1].toFloat();
      float z = pieces[2].toFloat();
      float ga = map(z, 0, 300, 0, 90);
      doArmIKLimits(x, y, z, ga);

      // Clear out string and counters to get ready for the next incoming string
      input = "";
      counter = 0;
      lastIndex = 0;
    }
    else {
      // If we havent reached newline yet, add current character to string
      input += ch;
    }
  }
}
