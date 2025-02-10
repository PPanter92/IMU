#include "led_and_buttons.h"
#include "ultrasonic_sensor.h"
#include "motor_controller.h"
#include "webserver_ms.h"
#include "odometry.h"
#include "pid.h"

#define LED_PIN 11
#define BL_PIN 2
#define BR_PIN 3
#define US_TRIGGER_PIN 27
#define US_ECHO_PIN 0

#define TURNED_OFF 0
#define HORIZONTAL_MODE 1
#define VERTICAL_MODE 2

static uint8_t driveMode = TURNED_OFF;

volatile bool buttonRPressed = false;
volatile bool buttonLPressed = false;
static float wsSendData[8] = { 0.0f };
static float wsSteerData[2] = { 0.0f };
static float wsPidData[9] = { 0.0f };
bool steerDataNew = false;
bool pidDataNew = false;

void setup() {
  char ssid[] = "Temp";
  char password[] = "1234567890";

  initWebsocketServer(ssid, password, false);
  while (1) {
    websocketRefresh();
    websocketSendData(wsSendData, 100);
    if (websocketGetSteerData(wsSteerData) == true) {
      steerDataNew = true;
    }
    if (websocketGetPidData(wsPidData) == true) {
      pidDataNew = true;
    }
  }
}

void loop() {
}

void setup1() {
  Serial.begin(115200);
  initLedAndButtons(LED_PIN, BL_PIN, BR_PIN);
  initUsSensor(US_TRIGGER_PIN, US_ECHO_PIN, 500);
  initMotorDriver();
  initOdometry();
  static float speedVals[2] = {0.0f}, odomVals[3] = {0.0f};
  static float alphaVal = 0.0f, targetAngle = 0.0f;

  PID_INTERVAL pidParaAngle = {
      .kp = 12.0f, .ki = 125.0f, .kd = 0.0f,
      .lowerLimit = -255, .upperLimit = 255,
      .reverse = true,
      .lastUpdate = 0,
      .integral = 0.0f,
      .previousError = 0.0f
  };

  PID_INTERVAL pidParaSpeed = {
      .kp = 0.01f, .ki = 0.0075f, .kd = 0.0f,
      .lowerLimit = -20, .upperLimit = 20,
      .reverse = false,
      .lastUpdate = 0,
      .integral = 0.0f,
      .previousError = 0.0f
  };

  while (1) {
    if (buttonLPressed && !buttonRPressed) driveMode = HORIZONTAL_MODE;
    else if (!buttonLPressed && buttonRPressed) driveMode = VERTICAL_MODE;
    else driveMode = TURNED_OFF;

    wsSendData[1] = getBatteryVoltage(100) / 1000.0f;
    if (newDistanceUs() == true) {
      uint16_t distanceUs = getDistanceUs();
      wsSendData[0] = distanceUs / 100.0f;
    }
    if (getSimpleSpeed(speedVals, 100) == true) {
      wsSendData[2] = speedVals[0];
      wsSendData[3] = speedVals[1];
    }
    if (getOdom(odomVals, 100) == true) {
      wsSendData[4] = odomVals[0];
      wsSendData[5] = odomVals[1];
      wsSendData[6] = odomVals[2];
      Serial.print(odomVals[0]);
      Serial.print(", ");
      Serial.print(odomVals[1]);
      Serial.print(", ");
      Serial.println(odomVals[2]);
    }
    if (getSimpleOdom(odomVals, 100) == true) {
      wsSendData[4] = odomVals[0];
      wsSendData[5] = odomVals[1];
      wsSendData[6] = odomVals[2];
      Serial.print(odomVals[0]);
      Serial.print(", ");
      Serial.print(odomVals[1]);
      Serial.print(", ");
      Serial.println(odomVals[2]);
    }
    if (getAlphaAngle(alphaVal, 0.98, 0) == true) {
      wsSendData[7] = alphaVal;
    }

    if (driveMode == TURNED_OFF) {
      fadeLedNonBlocking(LED_PIN, 20);
      int16_t motorPwm[2] = {0, 0};
      setMotorPwm(motorPwm);
      pidParaAngle.integral = 0.0f;
      pidParaAngle.previousError = 0.0f;
      pidParaSpeed.integral = 0.0f;
      pidParaSpeed.previousError = 0.0f;

      targetAngle = alphaVal;
    }
    else if (driveMode == HORIZONTAL_MODE) {
      blinkLedNonBlocking(LED_PIN, 250);
      if (steerDataNew) {
        steerDataNew = false;
        static int16_t motorPwm[2] = { 0 };
        steerWheelchairMode(wsSteerData, motorPwm);
        setMotorPwm(motorPwm);
      }
    }
    else if (driveMode == VERTICAL_MODE) {
      fadeLedNonBlocking(LED_PIN, 5);
    
      float offsetAngle = pidController(0, speedVals[0], pidParaSpeed, 100);
      if (!isnan(offsetAngle)) {
        targetAngle += offsetAngle;
      }

      float motorOutput = pidController(targetAngle, alphaVal, pidParaAngle, 50);
      if (!isnan(motorOutput)) {
        int16_t motorPwm[2] = {(int16_t)motorOutput, (int16_t)motorOutput};
        setMotorPwm(motorPwm);
      }
    }

    if (pidDataNew == true) {
      pidDataNew = false;
      pidParaAngle.kp = wsPidData[0];
      pidParaAngle.ki = wsPidData[1];
      pidParaAngle.kd = wsPidData[2];

      pidParaSpeed.kp = wsPidData[3];
      pidParaSpeed.ki = wsPidData[4];
      pidParaSpeed.kd = wsPidData[5];
    }
  }
}

void loop1() {
}
