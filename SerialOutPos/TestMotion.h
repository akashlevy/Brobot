#ifndef TESTMOTION_H
#define TESTMOTION_H

#include "Kinematics.h"
#include "GlobalArm.h"


//=============================================================================
// Test Functions
//=============================================================================

/* Tests x = 0 */
void zero_x()
{
  for(double yaxis = 150.0; yaxis < 200.0; yaxis += 1){
    doArmIK(true, 0, yaxis, 100.0, 0);
    SetServo(0);
    delay(10);
  }
  for(double yaxis = 200.0; yaxis > 150.0; yaxis -= 1){
    doArmIK(true, 0, yaxis, 100.0, 0);
    SetServo(0);
    delay(10);
  }
}
 
/* Moves arm in a straight line */
void line()
{
    for(double xaxis = -100.0; xaxis < 100.0; xaxis += 0.5){
      doArmIK(true, xaxis, 200, 100, 0);
      SetServo(0);
      delay(10);
    }
    for(float xaxis = 100.0; xaxis > -100.0; xaxis -= 0.5){
      doArmIK(true, xaxis, 200, 100, 0);
      SetServo(0);
      delay(10);
    }
}
 
/* Moves arm in a circle */
void circle()
{
  #define RADIUS 20.0
  float zaxis, yaxis;
  for(float angle = 0.0; angle < 360.0; angle += 1.0){
      yaxis = RADIUS * sin(radians(angle)) + 150;
      zaxis = RADIUS * cos(radians(angle)) + 150;
      doArmIK(true, 0, yaxis, zaxis, 0);
      SetServo(0);
      delay(5);
  }
}

#endif
