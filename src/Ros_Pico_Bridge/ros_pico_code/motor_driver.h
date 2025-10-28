#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "Arduino.h"

#define PWM_LEFT  10
#define DIR_LEFT  11
#define PWM_RIGHT 12
#define DIR_RIGHT 13

#define MAX_PWM 255

// Declare variables defined elsewhere (in diff_controller.h)
extern int moving;
extern struct PIDState leftPID;
extern struct PIDState rightPID;

void initMotorController() {
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  // Left side
  if (leftSpeed >= 0) {
    digitalWrite(DIR_LEFT, HIGH);
    analogWrite(PWM_LEFT, leftSpeed);
  } else {
    digitalWrite(DIR_LEFT, LOW);
    analogWrite(PWM_LEFT, -leftSpeed);
  }

  // Right side
  if (rightSpeed >= 0) {
    digitalWrite(DIR_RIGHT, HIGH);
    analogWrite(PWM_RIGHT, rightSpeed);
  } else {
    digitalWrite(DIR_RIGHT, LOW);
    analogWrite(PWM_RIGHT, -rightSpeed);
  }
}

#endif
