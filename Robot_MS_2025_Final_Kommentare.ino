#include "led_and_buttons.h"
#include "ultrasonic_sensor.h"
#include "motor_controller.h"
#include "webserver_ms.h"
#include "odometry.h"
#include "pid.h"

// Pin definitions
#define LED_PIN 11
#define BL_PIN 2
#define BR_PIN 3
#define US_TRIGGER_PIN 27
#define US_ECHO_PIN 0

// Drive mode states
#define TURNED_OFF 0
#define HORIZONTAL_MODE 1
#define VERTICAL_MODE 2

static uint8_t driveMode = TURNED_OFF; // Current driving mode

// Global variables for button states
volatile bool buttonRPressed = false;
volatile bool buttonLPressed = false;

// WebSocket data buffers
static float wsSendData[8] = { 0.0f }; // Data to send over WebSocket
static float wsSteerData[2] = { 0.0f }; // Steering input data from WebSocket
static float wsPidData[9] = { 0.0f }; // PID parameters received via WebSocket

bool steerDataNew = false; // Flag indicating new steering data
bool pidDataNew = false; // Flag indicating new PID data

/**
 * Initializes the WebSocket server and continuously updates data.
 */
void setup() {
  // Wi-Fi credentials
  char ssid[] = "ssid";
  char password[] = "pw";

  initWebsocketServer(ssid, password, false);

  // Infinite loop to handle WebSocket communication
  while (1) {
    websocketRefresh(); // Refresh WebSocket connection
    websocketSendData(wsSendData, 100); // Send sensor data over WebSocket

    // Check if new steering data is received
    if (websocketGetSteerData(wsSteerData) == true) {
      steerDataNew = true;
    }
    // Check if new PID data is received
    if (websocketGetPidData(wsPidData) == true) {
      pidDataNew = true;
    }
  }
}

void loop() {
  // Empty loop, as all processing happens in `setup1()`
}

/**
 * Initializes all necessary peripherals and handles control logic.
 */
void setup1() {
  Serial.begin(115200);

  // Initialize hardware components
  initLedAndButtons(LED_PIN, BL_PIN, BR_PIN);
  initUsSensor(US_TRIGGER_PIN, US_ECHO_PIN, 500);
  initMotorDriver();
  initOdometry();

  // Sensor and control variables
  static float speedVals[2] = {0.0f}, odomVals[3] = {0.0f};
  static float alphaVal = 0.0f, targetAngle = 0.0f;

  // PID controller for angle stabilization
  PID_INTERVAL pidParaAngle = {
      .kp = 12.0f, .ki = 125.0f, .kd = 0.0f,
      .lowerLimit = -255, .upperLimit = 255,
      .reverse = true,
      .lastUpdate = 0,
      .integral = 0.0f,
      .previousError = 0.0f
  };

  // PID controller for speed stabilization
  PID_INTERVAL pidParaSpeed = {
      .kp = 0.01f, .ki = 0.0075f, .kd = 0.0f,
      .lowerLimit = -20, .upperLimit = 20,
      .reverse = false,
      .lastUpdate = 0,
      .integral = 0.0f,
      .previousError = 0.0f
  };

  // Main control loop
  while (1) {
    // Determine the current driving mode based on button states
    if (buttonLPressed && !buttonRPressed) driveMode = HORIZONTAL_MODE;
    else if (!buttonLPressed && buttonRPressed) driveMode = VERTICAL_MODE;
    else driveMode = TURNED_OFF;

    // Update WebSocket data with battery voltage
    wsSendData[1] = getMeanBatteryVoltage(100) / 1000.0f;

    // Read ultrasonic distance sensor
    if (newDistanceUs() == true) {
      uint16_t distanceUs = getDistanceUs();
      wsSendData[0] = distanceUs / 100.0f;
    }

    // Read wheel speeds
    if (getSimpleSpeed(speedVals, 100) == true) {
      wsSendData[2] = speedVals[0];
      wsSendData[3] = speedVals[1];
    }

    // Read odometry data
    if (getSimpleOdom(odomVals, 100) == true) {
      wsSendData[4] = odomVals[0];
      wsSendData[5] = odomVals[1];
      wsSendData[6] = odomVals[2];
    }

    // Read inclination angle
    if (getAlphaAngle(alphaVal, 0.98, 0) == true) {
      wsSendData[7] = alphaVal;
    }

    // Handle different driving modes
    if (driveMode == TURNED_OFF) {
      // LED fade effect to indicate standby mode
      fadeLedNonBlocking(LED_PIN, 20);

      // Stop motors and reset PID controllers
      int16_t motorPwm[2] = {0, 0};
      setMotorPwm(motorPwm);
      pidParaAngle.integral = 0.0f;
      pidParaAngle.previousError = 0.0f;
      pidParaSpeed.integral = 0.0f;
      pidParaSpeed.previousError = 0.0f;

      // Store current angle as reference for next movement
      targetAngle = alphaVal;
    }
    else if (driveMode == HORIZONTAL_MODE) {
      // Blink LED to indicate horizontal movement mode
      blinkLedNonBlocking(LED_PIN, 250);

      // Process new steering data and update motors
      if (steerDataNew) {
        steerDataNew = false;
        static int16_t motorPwm[2] = { 0 };
        steerHorizontalMode(wsSteerData, motorPwm);
        setMotorPwm(motorPwm);
      }
    }
    else if (driveMode == VERTICAL_MODE) {
      // LED fade effect for vertical mode
      fadeLedNonBlocking(LED_PIN, 5);
    
      // Speed PID control to adjust target angle dynamically
      float offsetAngle = pidController(0, speedVals[0], pidParaSpeed, 100);
      if (!isnan(offsetAngle)) {
        targetAngle += offsetAngle;
      }

      // Angle PID control to stabilize the system
      float motorOutput = pidController(targetAngle, alphaVal, pidParaAngle, 50);
      if (!isnan(motorOutput)) {
        int16_t motorPwm[2] = {(int16_t)motorOutput, (int16_t)motorOutput};
        setMotorPwm(motorPwm);
      }
    }

    // Update PID parameters if new values are received via WebSocket
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

/**
 * Empty loop function, as all processing occurs in `setup1()`.
 */
void loop1() {
}
