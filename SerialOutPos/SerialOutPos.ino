/***********************************************************************************
 *  }--\     RobotGeek Snapper Robotic Arm     /--{
 *      |       Analog IK Control Code        |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will move allow you to contro the Snapper Robot Arm using
 *  joysticks and a knob.
 *
 *  By setting the control mode you can control the arm in joint/backhoe mode,
 *  Cartesian IK or Cylindric
 * the arm to an X/Y/Z coordinate based on the inputs
 *  from the analog inputs (joysticks and knob). This sketch can also be used to play
 *  back a pre-programmed sequence.
 *
 *  Snapper Arm Getting Started Guide
 *   http://learn.robotgeek.com/getting-started/33-robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide.html
 *  Using the IK Firmware
 *    http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html
 *
 *
 *  WIRING
 *    Servos
 *      Digital I/O 3 - Base Rotation - Robot Geek Servo
 *      Digital I/O 5 - Shoulder Joint - Robot Geek Servo
 *      Digital I/O 6 - Elbow Joint - Robot Geek Servo
 *      Digital I/O 9 - Wrist Joint - Robot Geek Servo
 *      Digital I/O 10 - Gripper Servo - 9g Servo
 *
 *    Analog Inputs
 *      Analog 0 - Joystick (Horizontal)
 *      Analog 1 - Joystick (Vertical)
 *      Analog 2 - Joystick (Vertical)
 *      Analog 3 - Joystick (Vertical)
 *      Analog 4 - Rotation Knob
 *
 *    Digital Inputs
 *      Digital 2 - Button 1
 *      Digital 4 - Button 2
 *
 *
 *    Use an external power supply and set both PWM jumpers to 'VIN'
 *
 *  CONTROL
 *      Analog 0 - Joystick - Control the Y Axis (forward/back)
 *      Analog 1 - Joystick - Control the X Axis (left/right)
 *      Analog 2 - Joystick - Control the Z Axis (up/down)
 *      Analog 3 - Joystick - Control the Wrist Angle
 *      Analog 4 - Rotation Knob - Control the Gripper
 *    http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html
 *
 *
 *
 *  NOTES
 *
 *    SERVO POSITIONS
 *      The servos' positions will be tracked in microseconds, and written to the servos
 *      using .writeMicroseconds()
 *        http://arduino.cc/en/Reference/ServoWriteMicroseconds
 *      For RobotGeek servos, 600ms corresponds to fully counter-clockwise while
 *      2400ms corresponds to fully clock-wise. 1500ms represents the servo being centered
 *
 *      For the 9g servo, 900ms corresponds to fully counter-clockwise while
 *      2100ms corresponds to fully clock-wise. 1500ms represents the servo being centered
 *
 *
 *  This code is a Work In Progress and is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 * Sources used:
 * https://github.com/KurtE
 *
 * http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 *
 * Application Note 44 - Controlling a Lynx6 Robotic Arm
 * http://www.micromegacorp.com/appnotes.html
 * http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 *
 *
 *   This code is a Work In Progress and is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 *
 ***********************************************************************************/
//#define DEFAULT_CONTROL_MODE  IKM_BACKHOE
#define DEFAULT_CONTROL_MODE  IKM_IK3D_CARTESIAN
//#define DEFAULT_CONTROL_MODE  IKM_CYLINDRICAL



#define ROBOT_GEEK_9G_GRIPPER 1
#define ROBOT_GEEK_PARALLEL_GRIPPER 2

//The 9G gripper is the gripper with the small blue 9g servo
//The Parralle gripper has a full robotgeek servo and paralle rails
//Uncomment one of the following lines depending on which gripper you are using.
//#define GRIPPER_TYPE ROBOT_GEEK_9G_GRIPPER
#define GRIPPER_TYPE ROBOT_GEEK_PARALLEL_GRIPPER

#ifndef GRIPPER_TYPE
   #error YOU HAVE TO SELECT THE GRIPPER YOU ARE USING! Uncomment the correct line above for your gripper
#endif

#include "ServoEx.h"
#include "InputControl.h"

ServoEx    ArmServo[5];

//===================================================================================================
// Setup
//===================================================================================================
void setup(){
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(5, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(7, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(9, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(11, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(13, GRIPPER_MIN, GRIPPER_MAX);

  // Start serial
  Serial.begin(9600);
  delay(500);
}

// Define number of pieces
const int numberOfPieces = 4;
String pieces[numberOfPieces];

// This will be the buffered string from Serial.read()
// up until you hit a \n
// Should look something like "123,456,789,0"
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
          // Grab the piece from the last index up to the current position and store it
          pieces[counter] = input.substring(lastIndex, i);
          // Update the last position and add 1, so it starts from the next character
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
      g_sIKX = pieces[0].toFloat();
      g_sIKY = pieces[1].toFloat();
      g_sIKZ = pieces[2].toFloat();
      g_sIKGA = pieces[3].toFloat();
      doArmIK(true, g_sIKX,g_sIKY,g_sIKZ,g_sIKGA);
      MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
      SetServo(0);
      

      // Clear out string and counters to get ready for the next incoming string
      input = "";
      counter = 0;
      lastIndex = 0;
    }
    else {
      // If we havent reached a newline character yet, add the current character to the string
      input += ch;
    }
  }
}
