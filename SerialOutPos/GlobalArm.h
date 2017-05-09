#ifndef GLOBALARM_H
#define GLOBALARM_H

// Motion limits (mm)
#define IK_MAX_X 150
#define IK_MIN_X -150

#define IK_MAX_Y 200
#define IK_MIN_Y 50

#define IK_MAX_Z 275
#define IK_MIN_Z 80

#define IK_MAX_GA 30
#define IK_MIN_GA -30

// Arm dimensions (mm)
#define BASE_HGT      88.75   // base height
#define HUMERUS       96.00   // shoulder-to-elbow "humerus bone"
#define ULNA          90.6    // elbow-to-wrist "ulna bone"
#define GRIPPER       96      // wrist-gripper length


//=============================================================================
// SERVO CONFIG
//=============================================================================

// Declare servos
enum { BAS_SERVO, SHL_SERVO, ELB_SERVO, WRI_SERVO, GRI_SERVO };

// Servo position limitations - limits in microseconds
#define BASE_MIN      600     // full CCW for RobotGeek 180 degree servo
#define BASE_MAX      2400    // full CW for RobotGeek 180 degree servo
#define SHOULDER_MIN  600
#define SHOULDER_MAX  2400
#define ELBOW_MIN     600
#define ELBOW_MAX     2400
#define WRIST_MIN     600
#define WRIST_MAX     2400
#define GRIPPER_MIN   750    // fully closed
#define GRIPPER_MAX   2400   // fully open

// Define servo offsets in +/- uS
#define BAS_SERVO_ERROR 0 // (+ is CW, - is CCW)
#define SHL_SERVO_ERROR 0 // (+ is forward, - is backward)
#define ELB_SERVO_ERROR 0 // (+ is up, - is down)
#define WRI_SERVO_ERROR 0 // (+ is up, - is down)
#define GRI_SERVO_ERROR 0 // (+ is tighten grip, - is loosen grip) 

// Present positions of the servos 
float Base     = 1500;
float Shoulder = 1500;
float Elbow    = 1500;
float Wrist    = 1500;
int   Gripper  = 1500;

#endif
