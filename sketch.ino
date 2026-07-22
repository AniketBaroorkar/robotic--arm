#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Default PCA9685 I2C address
Adafruit_PWMServoDriver pwm =
    Adafruit_PWMServoDriver(0x40);

// Robotic-arm servo channels
constexpr uint8_t BASE_SERVO = 0;
constexpr uint8_t SHOULDER_SERVO = 1;
constexpr uint8_t ELBOW_SERVO = 2;
constexpr uint8_t WRIST_PITCH_SERVO = 3;
constexpr uint8_t WRIST_ROTATION_SERVO = 4;
constexpr uint8_t GRIPPER_SERVO = 5;

constexpr uint8_t SERVO_CHANNELS[6] = {
    BASE_SERVO,
    SHOULDER_SERVO,
    ELBOW_SERVO,
    WRIST_PITCH_SERVO,
    WRIST_ROTATION_SERVO,
    GRIPPER_SERVO
};

// Standard positional servos normally use 50 Hz.
constexpr uint16_t SERVO_FREQUENCY = 50;

// Approximately the centre position of a positional servo.
constexpr uint16_t CENTER_PULSE_US = 1500;

/*
  At 50 Hz, one complete PWM period is 20,000 microseconds.

  PCA9685 divides that period into 4096 steps.
*/
uint16_t microsecondsToTicks(uint16_t microseconds) {
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(microseconds) * 4096UL)
        / 20000UL
    );
}

void moveAllServosTo90() {
    const uint16_t centerTick =
        microsecondsToTicks(CENTER_PULSE_US);

    // Move one servo at a time to reduce the startup current surge.
    for (uint8_t index = 0; index < 6; index++) {
        const uint8_t channel =
            SERVO_CHANNELS[index];

        pwm.setPWM(
            channel,
            0,
            centerTick
        );

        Serial.print("Channel ");
        Serial.print(channel);
        Serial.println(" moved to approximately 90 degrees.");

        delay(500);
    }
}

void setup() {
    Serial.begin(115200);

    // Start the UNO Q MCU I2C interface.
    Wire.begin();

    // Start the PCA9685.
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQUENCY);

    delay(500);

    Serial.println("Moving robotic arm to HOME...");
    moveAllServosTo90();
    Serial.println("HOME completed: all six channels are at 90 degrees.");
}

void loop() {
    // Do nothing.
    // The PCA9685 continues holding the last servo positions.
}
