#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

InputController inputController;
LEDController ledController;
SensorController sensorController;
MotionController motionController;
HIDController hidController;
TelemetryController telemetryController;

void setup() {
  // Initialize USB HID first
  hidController.begin();

  if (Config::ENABLE_TELEMETRY) {
    Serial.begin(115200);
    delay(200);
  }

delay(1000);
//Serial.println(">start!!!!!A:");
//Serial.println(">start!!!!!A:");

  inputController.begin();
  ledController.begin();

Serial.println(">sensor begin");
  sensorController.begin();

Serial.println(">motion begin");
  motionController.reset();

Serial.println(">telemetry begin");
  telemetryController.begin();

Serial.println(">calibrating");
  stateMachine.changeState(&StateMachine::calibratingState);

  //Serial.println(">I2C1:");
  //Serial.println(SDA);
  //Serial.println(SCL);
}

void loop() {
  hidController.task();
  stateMachine.update();
  //Serial.println(">main loop:");
}
