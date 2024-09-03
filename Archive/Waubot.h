#pragma once

#include <stdint.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "QTRSensors_mux.h"

#define SENSOR_CNT 14
#define ADDR_BASE 10
#define CENTER_VAL 6500
#define MAX_SPEED 180
#define MIN_SPEED 0

// Moving types.
enum class ActionType : uint8_t {
  FORWARD,
  BACKWARD,
  RIGHT,
  LEFT,
  RIGHT_AT_CENTER,
  LEFT_AT_CENTER
};

// Junction types.
enum JunctionType {
  CENTER_T,
  CENTER_LEFT,
  CENTER_RIGHT,
  LEFT_ONLY,
  RIGHT_ONLY,
  CENTER_Y
};

enum LineFormat {
  BLACK_LINE,
  WHITE_LINE
};

enum GripType {
  RELEASE,
  GRIP
};

static int _countH1, _countH2, _countH3, _countH4;

class Waubot {

public:

  Waubot(bool debug);

  ~Waubot();

  void moveDelay(LineFormat lineFormat, int speedBase, float kp, float kd, int delay);

  void moveJunction(
    JunctionType junc,
    ActionType action,
    LineFormat line,
    int forwardDelay,
    int forwardSpeed,
    int actionDelay,
    int actionSpeed,
    float kp, float kd);

  void moveForwardDelay(int sp, int dly);

  void moveBackwardDelay(int sp, int dly);

  void moveForwardCustomDelay(int lsp, int rsp, int dly);

  void moveBackwardCustomDelay(int lsp, int rsp, int dly);

  void stop();

  void calibrate();

  int readBtnEvent(int idx);

private:

  Adafruit_NeoPixel _pixels;

  QTRSensors _qtr;

  int _lastError;

  uint16_t _sensorValues[SENSOR_CNT];

  uint16_t _sensorAvg[SENSOR_CNT];

  bool _isDebug = false;

  void pid(LineFormat lineFormat, int speedBase, float Kp, float Kd);

  void motorForward(int sp);

  void motorBackward(int sp);

  void motorForwardDrive(int lsp, int rsp);

  void motorBackwardDrive(int lsp, int rsp);

  void motorTurnAtCenter(ActionType action, int sp);

  void motorStop();

  static void isrCountH1() {
    if (_countH1 == NULL) _countH1 = 0;
    _countH1++;
  }

  static void isrCountH2() {
    if (_countH2 == NULL) _countH2 = 0;
    _countH2++;
  }

  static void isrCountH3() {
    if (_countH3 == NULL) _countH3 = 0;
    _countH3++;
  }

  static void isrCountH4() {
    if (_countH4 == NULL) _countH4 = 0;
    _countH4++;
  }
};
