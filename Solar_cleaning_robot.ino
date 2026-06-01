#include "driver/ledc.h"  // ESP32 LEDC hardware PWM

// Motor driver pins
const int leftMotorPin1 = 21;
const int leftMotorPin2 = 19;
const int rightMotorPin1 = 33;
const int rightMotorPin2 = 32;

const int leftEnablePin = 18;
const int rightEnablePin = 25;

// IR sensor pin (digital)
const int irSensorPin = 23;  // Digital OUT of IR sensor

// PWM settings
const int freq = 30000;
const int resolution = LEDC_TIMER_8_BIT;
const uint32_t dutyCycle = 190;  // Motor speed (0-255)

const ledc_channel_t leftPWMChannel = LEDC_CHANNEL_0;
const ledc_channel_t rightPWMChannel = LEDC_CHANNEL_1;

// Timing constants
const int TURN_DURATION = 5000;
const int BACKWARD_TIME = 2000;
const int STATE_DELAY = 200;

enum RobotState {
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING_RIGHT,
  STOPPED
};

RobotState currentState = MOVING_FORWARD;
unsigned long stateStartTime = 0;
bool edgeDetected = false;

void setupPWM(ledc_channel_t channel, int pin) {
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = (ledc_timer_bit_t)resolution,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = freq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num = pin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = channel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(irSensorPin, INPUT_PULLUP);  // Enable internal pull-up

  setupPWM(leftPWMChannel, leftEnablePin);
  setupPWM(rightPWMChannel, rightEnablePin);

  Serial.println("Robot Initialized. Starting...");
}

void loop() {
  int irValue = digitalRead(irSensorPin);
  Serial.print("IR Value: ");
  Serial.println(irValue);

  switch (currentState) {
    case MOVING_FORWARD:
      moveForward();
      if (irValue == HIGH) {  // Edge detected (assuming HIGH means edge)
        Serial.println("Edge Detected → Stopping");
        stopMotors();
        delay(STATE_DELAY);
        currentState = MOVING_BACKWARD;
        stateStartTime = millis();
      }
      break;

    case MOVING_BACKWARD:
      moveBackward();
      if (millis() - stateStartTime >= BACKWARD_TIME) {
        stopMotors();
        delay(STATE_DELAY);
        currentState = TURNING_RIGHT;
        stateStartTime = millis();
      }
      break;

    case TURNING_RIGHT:
      turnRight();
      if (millis() - stateStartTime >= TURN_DURATION) {
        stopMotors();
        delay(STATE_DELAY);
        currentState = MOVING_FORWARD;  // Return to forward movement
      }
      break;

    case STOPPED:
      // Do nothing
      break;
  }

  delay(50);  // Small delay to prevent flooding serial monitor
}

// ================= MOTOR CONTROL FUNCTIONS ===================

void moveForward() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  setMotorSpeed(dutyCycle);
}

void moveBackward() {
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  setMotorSpeed(dutyCycle);
}

void turnRight() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  setMotorSpeed(dutyCycle);
}

void stopMotors() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  setMotorSpeed(0);
}

void setMotorSpeed(uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, leftPWMChannel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, leftPWMChannel);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, rightPWMChannel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, rightPWMChannel);
}