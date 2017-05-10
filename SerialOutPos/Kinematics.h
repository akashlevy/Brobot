#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "GlobalArm.h"
#include <Arduino.h>


//=============================================================================
// Global Variables
//=============================================================================

extern ServoEx ArmServo[5];


//=============================================================================
// Miscellaneous Definitions
//=============================================================================

// Float-to-long conversion
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

// Old/Current IK values
float            sIKX  = 0.00;
float            sIKY  = 150.00;
float            sIKZ  = 150.00;
float            sIKGA = 0.00;

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
  float Base = (ftl(1500.0 - (( degrees( bas_angle_r )) * 10.55 )));
  float Shoulder = (ftl(1500.0 - (( shl_angle_d - 90) * 10.55 )));
  float Elbow = (ftl(1500.0 + (( elb_angle_d - 90.0 ) * 10.55 )));
  float Wrist = (ftl(1500 + ( wri_angle_d  * 10.55 )));

  // Move the servos
  ServoGroupMove.start();
  ArmServo[BAS_SERVO].writeMicroseconds(Base + BAS_SERVO_ERROR);
  ArmServo[SHL_SERVO].writeMicroseconds(Shoulder + SHL_SERVO_ERROR);
  ArmServo[ELB_SERVO].writeMicroseconds(Elbow + ELB_SERVO_ERROR);
  ArmServo[WRI_SERVO].writeMicroseconds(Wrist + WRI_SERVO_ERROR);
  ServoGroupMove.commit(0);

  // Save old values
  sIKX  = x;
  sIKY  = y;
  sIKZ  = z;
  sIKGA = grip_angle_d;
}

void doArmIKLimits(float x, float y, float z, float grip_angle_d) {
  // Restrict range
  x = max(min(x, IK_MAX_X), IK_MIN_X);
  y = max(min(y, IK_MAX_Y), IK_MIN_Y);
  z = max(min(z, IK_MAX_Z), IK_MIN_Z);
  
  // Get the updated angles
  doArmIK(x, y, z, grip_angle_d);
}

#endif




