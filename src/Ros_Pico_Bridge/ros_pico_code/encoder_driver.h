// ========================== encoder_driver.h ==========================
#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include "Arduino.h"

// Encoder pins
#define LEFT_ENC_A  2
#define LEFT_ENC_B  3
#define RIGHT_ENC_A 4
#define RIGHT_ENC_B 5

volatile long left_enc_count = 0;
volatile long right_enc_count = 0;

// Interrupt routines
void leftEncoderA() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B))
    left_enc_count++;
  else
    left_enc_count--;
}

void rightEncoderA() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B))
    right_enc_count++;
  else
    right_enc_count--;
}

void initEncoders() {
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderA, CHANGE);
}

long readEncoder(int side) {
  if (side == 0)
    return left_enc_count;
  else
    return right_enc_count;
}

void resetEncoders() {
  left_enc_count = 0;
  right_enc_count = 0;
}

#endif
