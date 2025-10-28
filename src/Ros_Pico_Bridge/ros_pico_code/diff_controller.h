#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

#include "motor_driver.h"
#include "encoder_driver.h"

#define LEFT  0
#define RIGHT 1

float Kp = 20.0;
float Ki = 0.0;
float Kd = 0.0;
float Ko = 50.0;

// Only defined here (not extern)
struct PIDState {
  long TargetTicksPerFrame;
  long Encoder;
  long PrevEnc;
  long Output;
  long PrevInput;
  long ITerm;
};

PIDState leftPID, rightPID;
int moving = 0;

void resetPID() {
  leftPID.TargetTicksPerFrame = 0;
  rightPID.TargetTicksPerFrame = 0;
  leftPID.Encoder = leftPID.PrevEnc = leftPID.Output = leftPID.PrevInput = leftPID.ITerm = 0;
  rightPID.Encoder = rightPID.PrevEnc = rightPID.Output = rightPID.PrevInput = rightPID.ITerm = 0;
}

void updatePID() {
  if (!moving) return;

  long encLeft = readEncoder(LEFT);
  long encRight = readEncoder(RIGHT);

  long inputLeft = encLeft - leftPID.PrevEnc;
  long inputRight = encRight - rightPID.PrevEnc;

  long errorLeft = leftPID.TargetTicksPerFrame - inputLeft;
  long errorRight = rightPID.TargetTicksPerFrame - inputRight;

  leftPID.ITerm += Ki * errorLeft;
  rightPID.ITerm += Ki * errorRight;

  long outputLeft = (Kp * errorLeft - Kd * (inputLeft - leftPID.PrevInput) + leftPID.ITerm) / Ko;
  long outputRight = (Kp * errorRight - Kd * (inputRight - rightPID.PrevInput) + rightPID.ITerm) / Ko;

  leftPID.Output += outputLeft;
  rightPID.Output += outputRight;

  leftPID.Output = constrain(leftPID.Output, -MAX_PWM, MAX_PWM);
  rightPID.Output = constrain(rightPID.Output, -MAX_PWM, MAX_PWM);

  setMotorSpeeds(leftPID.Output, rightPID.Output);

  leftPID.PrevEnc = encLeft;
  rightPID.PrevEnc = encRight;
  leftPID.PrevInput = inputLeft;
  rightPID.PrevInput = inputRight;
}

#endif
