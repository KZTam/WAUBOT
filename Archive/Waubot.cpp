#include "Waubot_pin.h"
#include "Waubot.h"


/* Public Method */

Waubot::Waubot(bool debug) {

  _isDebug = debug;

  Serial.setDebugOutput(true);

  esp_log_level_set("*", ESP_LOG_VERBOSE);

  _pixels = Adafruit_NeoPixel(3, P_RGB, NEO_GRB + NEO_KHZ800);

  _pixels.begin();



  pinMode(P_S1_BTN, INPUT);
  pinMode(P_S3_BTN, INPUT);

  // // pinMode(P_BZR, OUTPUT);

  pinMode(P_PWMA, OUTPUT);
  pinMode(P_DIRA, OUTPUT);
  pinMode(P_PWMB, OUTPUT);
  pinMode(P_DIRB, OUTPUT);

  ledcSetup(0, 1000, 8);  //PWMA setup Channel 0
  ledcSetup(1, 1000, 8);  //PWMB Setup Channel 1
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(P_PWMA, 0);
  ledcAttachPin(P_PWMB, 0);


  pinMode(P_S0_ADC, OUTPUT);  //S0 ADC
  pinMode(P_S1_ADC, OUTPUT);  //S1 ADC
  pinMode(P_S2_ADC, OUTPUT);  //S2 ADC
  pinMode(P_S3_ADC, OUTPUT);  //S3 ADC

  // //interrupt on pin3
  // pinMode(P_H1, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(P_H1), isrCountH1, RISING);
  // pinMode(P_H2, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(P_H2), isrCountH2, RISING);
  // pinMode(P_H3, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(P_H3), isrCountH3, RISING);
  // pinMode(P_H4, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(P_H4), isrCountH4, RISING);

  motorStop();

  _qtr.setTypeAnalog();
  _qtr.setMuxPins((const uint8_t[]){ P_S0_ADC, P_S1_ADC, P_S2_ADC, P_S3_ADC }, P_SENSOR_VN, SENSOR_CNT);
  _qtr.releaseEmitterPins();

  _lastError = 0;
}

Waubot::~Waubot() {
}

void Waubot::moveDelay(LineFormat lineFormat, int speedBase, float kp, float kd, int delay) {
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < delay) {
    pid(lineFormat, speedBase, kp, kd);
  }
}

void Waubot::moveJunction(JunctionType junc, ActionType action, LineFormat line, int forwardDelay, int forwardSpeed,
                          int actionDelay, int actionSpeed, float kp, float kd) {
}

void Waubot::stop() {
  motorStop();
}

void Waubot::calibrate() {
  _qtr.resetCalibration();

  for (int i = 0; i < 150; i++) {
    _qtr.calibrate();
    delay(10);
  }
  for (int i = 0; i < SENSOR_CNT; i++) {
    _sensorAvg[i] = 500;
  }

}

int Waubot::readBtnEvent(int idx) {

  return idx == 1 ? digitalRead(P_S1_BTN) : digitalRead(P_S3_BTN);
}

/* Private Method */

void Waubot::pid(LineFormat lineFormat, int speedBase, float Kp, float Kd) {
  uint16_t position;
  if (lineFormat == BLACK_LINE) {
    position = _qtr.readLineBlack(_sensorValues);
  } else {
    position = _qtr.readLineWhite(_sensorValues);
  }

  int err = position - (int)CENTER_VAL;
  int corrSpeed = Kp * err + Kd * (err - _lastError);
  _lastError = err;

  motorForwardDrive((int)constrain(speedBase + corrSpeed, (int)MIN_SPEED, (int)MAX_SPEED),
                    (int)constrain(speedBase - corrSpeed, (int)MIN_SPEED, (int)MAX_SPEED));

  // for (int i = 0; i < SENSOR_CNT; i++) {
  //   Serial.print(_sensorValues[i]);
  //   Serial.print(", ");
  // }
  // Serial.println("");
  // Serial.print(position);
  // Serial.print(", ");
  // Serial.print(err);
  // Serial.print(", ");
  // Serial.print(corrSpeed);
  // Serial.print(", ");
  // Serial.print(_lastError);
  // Serial.println("");
}

void Waubot::moveForwardDelay(int sp, int dly) {
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < dly) {
    motorForward(sp);
  }
}

void Waubot::moveBackwardDelay(int sp, int dly) {
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < dly) {
    motorBackward(sp);
  }
}

void Waubot::moveForwardCustomDelay(int lsp, int rsp, int dly) {
  motorForwardDrive(lsp, rsp);
  delay(dly);
}

void Waubot::moveBackwardCustomDelay(int lsp, int rsp, int dly) {
  motorBackwardDrive(lsp, rsp);
  delay(dly);
}

void Waubot::motorForward(int sp) {
  motorForwardDrive(sp, sp);
}

void Waubot::motorBackward(int sp) {
  motorBackwardDrive(sp, sp);
}

void Waubot::motorForwardDrive(int lsp, int rsp) {
  Serial.print("motorForwardDrive - ");
  digitalWrite(P_DIRA, LOW);
  analogWrite(P_PWMA, rsp);
  digitalWrite(P_DIRB, HIGH);
  analogWrite(P_PWMB, lsp);

}

void Waubot::motorBackwardDrive(int lsp, int rsp) {
  Serial.println("motorBackwardDrive");
  digitalWrite(P_DIRA, HIGH);
  analogWrite(P_PWMA, rsp);
  digitalWrite(P_DIRB, LOW);
  analogWrite(P_PWMB, lsp);

}

void Waubot::motorTurnAtCenter(ActionType action, int sp) {
  Serial.println("motorTurnAtCenter");
  if (action == ActionType::RIGHT_AT_CENTER) {
    digitalWrite(P_DIRB, HIGH);
    analogWrite(P_PWMB, sp);
    digitalWrite(P_DIRA, HIGH);
    analogWrite(P_PWMA, sp);

  } else if (action == ActionType::LEFT_AT_CENTER) {
    digitalWrite(P_DIRB, LOW);
    analogWrite(P_PWMB, sp);
    digitalWrite(P_DIRA, LOW);
    analogWrite(P_PWMA, sp);
  } else {
    // Not supported
  }
}


void Waubot::motorStop() {
  Serial.println("motorStop");
  digitalWrite(P_DIRA, HIGH);
  analogWrite(P_PWMA, 0);
  digitalWrite(P_DIRB, HIGH);
  analogWrite(P_PWMB, 0);

}
