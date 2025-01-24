#ifndef Polaris_eFoundry_AGV_Repository_h
#define Polaris_eFoundry_AGV_Repository_h
#include <MeAuriga.h>
#include "Arduino.h"
#define BUZZER_PORT 45
#define SAFE_DISTANCE 20

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower lineFinder(PORT_9);
MeUltrasonicSensor ultrasonic(PORT_10);
MeBuzzer buzzer;
MeGyro gyro_ext(0, 0x68);
MeGyro gyro(1, 0x69);

int16_t MOVE_SPEED = 65;
int16_t MAX_SPEED = 100;
unsigned long LAST_BUZZER = 0;
const unsigned long BUZZER_INTERVAL = 500;
bool BUZZER_ON = false;
char LAST_TURN;

class robotState {
private:
  int _xAxis;
  int _yAxis;
  int _zAxis;
  char _lastTurn;
  int _objectDistance;
public:

  void getState() {
    _xAxis = gyro.getAngleX();
    _yAxis = gyro.getAngleY();
    _zAxis = gyro.getAngleZ();
    _lastTurn = LAST_TURN;
    _objectDistance = ultrasonic.distanceCm();
  }

  int get_xAxis() {
    return _xAxis;
  }
  int get_yAxis() {
    return _yAxis;
  }
  int get_zAxis() {
    return _zAxis;
  }
  char get_lastTurn() {
    return _lastTurn;
  }
  int getObjectDistance() {
    return _objectDistance;
  }
} ROBOT_STATE;

void Stop() {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

void Forward() {
  if (ultrasonic.distanceCm() > SAFE_DISTANCE) {
    Encoder_1.setMotorPwm(-MOVE_SPEED);
    Encoder_2.setMotorPwm(MOVE_SPEED);
  } else {
    Stop();
  }
}

void Forward(int16_t SPEED) {
  if (ultrasonic.distanceCm() > SAFE_DISTANCE) {
    Encoder_1.setMotorPwm(-SPEED);
  } else {
    Stop();
  }
}

void TurnLeft() {
  if (ultrasonic.distanceCm() > SAFE_DISTANCE) {
    Encoder_1.setMotorPwm(-MOVE_SPEED * 1.25);
    Encoder_2.setMotorPwm(MOVE_SPEED / 16);
  } else {
    Stop();
  }
}

void TurnLeft(int16_t SPEED) {
  if (ultrasonic.distanceCm() > SAFE_DISTANCE) {
    Encoder_1.setMotorPwm(-SPEED * 1.25);
    Encoder_2.setMotorPwm(SPEED / 16);
  } else {
    Stop();
  }
}

void TurnRight() {
  if (ultrasonic.distanceCm() > SAFE_DISTANCE) {
    Encoder_1.setMotorPwm(-MOVE_SPEED / 16);
    Encoder_2.setMotorPwm(MOVE_SPEED * 1.25);
  } else {
    Stop();
  }
}

void TurnRight(int16_t SPEED) {
  if (ultrasonic.distanceCm() > SAFE_DISTANCE) {
    Encoder_1.setMotorPwm(-SPEED / 16);
    Encoder_2.setMotorPwm(SPEED * 1.25);
  } else {
    Stop();
  }
}

void Drive() {
  int sensorState = lineFinder.readSensors();
  switch (sensorState) {
    case S1_IN_S2_IN:
      Forward();
      LAST_TURN = 'F';
      break;
    case S1_IN_S2_OUT:
      TurnLeft(); 
      LAST_TURN = 'L';
      break;
    case S1_OUT_S2_IN:
      TurnRight();
      LAST_TURN = 'R';
      break;
    case S1_OUT_S2_OUT:
      Stop();
      break;
  }

  if (ultrasonic.distanceCm() < SAFE_DISTANCE) {
    Stop();
    buzzer.tone(1000, 1000);
    buzzer.tone(600, 1000);
  } 
}

#endif