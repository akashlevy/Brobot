#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "GlobalArm.h"
#include <Arduino.h>


//=============================================================================
// Global Variables
//=============================================================================
extern ServoEx ArmServo[5];
boolean g_fArmActive = false;      // Is the arm logically on?
boolean g_fServosFree;             // Are the servos free?


//=============================================================================
// Miscellaneous Definitions
//=============================================================================

// Float-to-long conversion
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

// IK coordinate variables
// Current IK values
float            g_sIKX  = 0.00;
float            g_sIKY  = 150.00;
float            g_sIKZ  = 150.00;
float            g_sIKGA = 0.00;

// Next IK Values
float            sIKX  = 0.00;
float            sIKY  = 150.00;
float            sIKZ  = 150.00;
float            sIKGA = 0.00;

// Values for current servo values for the different joints
int             g_sBase = 1500;
int             g_sShoulder;
int             g_sElbow;
int             g_sWrist;
int             g_sWristRot;
int             g_sGrip;

int sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip = 1500;

//=============================================================================
// doArmIK: Floating Point Arm IK Solution for PWM Servos
// Arm positioning routine utilizing inverse kinematics
// z is height, y is distance from base center out, x is side to side
// y,z can only be positive
//=============================================================================
void doArmIK(float x, float y, float z, float grip_angle_d) {
  // Grip angle in radians for use in calculations
  float grip_angle_r = radians( grip_angle_d );

  // Base angle and radial distance from x,y coordinates
  float bas_angle_r = atan2( x, y );
  float rdist = sqrt(( x * x ) + ( y * y ));

  // rdist is y coordinate for the arm
  y = rdist;

  // Grip offsets calculated based on grip angle
  float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
  float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;

  // Wrist position
  float wrist_z = ( z - grip_off_z ) - BASE_HGT;
  float wrist_y = y - grip_off_y;

  // Shoulder to wrist distance
  float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
  float s_w_sqrt = sqrt( s_w );

  // Shoulder to wrist angle to ground
  float a1 = atan2( wrist_z, wrist_y );

  // Shoulder to wrist angle to humerus
  float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));

  // Shoulder angle
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees( shl_angle_r );

  // Elbow angle
  float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
  float elb_angle_d = degrees( elb_angle_r );
  float elb_angle_dn = -( 180.0 - elb_angle_d );

  // Wrist angle
  float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;
 
  // Servo pulses
  sBase = (ftl(1500.0 - (( degrees( bas_angle_r )) * 10.55 )));
  sShoulder = (ftl(1500.0 - (( shl_angle_d - 90) * 10.55 )));
  sElbow = (ftl(1500.0 + (( elb_angle_d - 90.0 ) * 10.55 )));
  sWrist = (ftl(1500 + ( wri_angle_d  * 10.55 )));
}

//=============================================================================
// MoveArmTo
//=============================================================================
void MoveArmTo(int sBase, int sShoulder, int sElbow, int sWrist, int sWristRot,
               int sGrip) {
  // Make sure servos are not free
  if (g_fServosFree) {
    g_fServosFree = false;
  }

  // Save the current positions of the joints
  g_sBase = sBase;
  g_sShoulder = sShoulder;
  g_sElbow = sElbow;
  g_sWrist = sWrist;
  g_sWristRot = sWristRot;
  g_sGrip = sGrip;
  Base = sBase;
  Shoulder = sShoulder;
  Elbow = sElbow;
  Wrist = sWrist;
  Gripper = sGrip;
}

//=============================================================================
// MoveArmToHome: Moves Arm to Home Position
//=============================================================================
void MoveArmToHome(void) {
    sBase = 1500;
    sShoulder = 1500;
    sElbow = 1500;
    sWrist = 1500;
    
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 256);
    g_fArmActive = false;
}

//=============================================================================
// SetServo: Writes Servo Position Solutions
//=============================================================================
void SetServo(unsigned int DeltaTime) {
  ServoGroupMove.start();
  ArmServo[BAS_SERVO].writeMicroseconds(Base + BAS_SERVO_ERROR);
  ArmServo[SHL_SERVO].writeMicroseconds(Shoulder + SHL_SERVO_ERROR);
  ArmServo[ELB_SERVO].writeMicroseconds(Elbow + ELB_SERVO_ERROR);
  ArmServo[WRI_SERVO].writeMicroseconds(Wrist + WRI_SERVO_ERROR);
  ArmServo[GRI_SERVO].writeMicroseconds(Gripper + GRI_SERVO_ERROR);
  ServoGroupMove.commit(DeltaTime);
}

void doArmIKLimits(float x, float y, float z, float grip_angle_d) {
  if (x > IK_MIN_X && x < IK_MAX_X && y > IK_MIN_Y && y < IK_MAX_Y &&
      z > IK_MIN_Z && z < IK_MAX_Z && grip_angle_d > IK_MIN_GA &&
      grip_angle_d < IK_MAX_GA) doArmIK(x, y, z, grip_angle_d);
}

#endif




