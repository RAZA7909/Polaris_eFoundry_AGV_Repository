#include "Polaris_eFoundry_AGV_Repository.h"

void setup() {
  Serial.begin(115200);
  buzzer.setpin(BUZZER_PORT);
  buzzer.tone(600, 1000);
  // Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // Ensure motors are stopped at startup
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

void loop() {
  int sensorState = lineFinder.readSensors();
  if (millis() - LAST_BUZZER >= BUZZER_INTERVAL) {
      LAST_BUZZER = millis();
      if (BUZZER_ON) {
        buzzer.noTone();
      } else {
        buzzer.tone(600, 150);
      }
      BUZZER_ON = !BUZZER_ON;
    }
  Drive();
}
