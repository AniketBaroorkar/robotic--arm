#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// Waveshare Servo Driver HAT = PCA9685 chip at I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const uint8_t NUM_SERVOS = 6;   // channels 0..5
bool hatFound = false;
// 90 degrees = 1500 us pulse. At 50 Hz with 4096 ticks:
// 1500us / 20000us * 4096 = 307 ticks
const uint16_t TICKS_90_DEG = 307;
void centerAllServos() {
 for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
   pwm.setPWM(ch, 0, TICKS_90_DEG);
 }
}
void setup() {
 Bridge.begin();          // required on UNO Q, do not remove
 Monitor.begin(115200);
 Wire.begin();
 delay(500);
 Monitor.println("=== Servo 90-degree test starting ===");
}
void loop() {
 // Ask the HAT if it is there (I2C address 0x40)
 Wire.beginTransmission(0x40);
 bool ok = (Wire.endTransmission() == 0);
 if (ok && !hatFound) {
   hatFound = true;
   Monitor.println("HAT FOUND at 0x40 -> all 6 servos going to 90");
   pwm.begin();
   pwm.setPWMFreq(50);    // standard servo frequency
   delay(10);
   centerAllServos();
 }
 if (!ok) {
   if (hatFound) Monitor.println("HAT lost! Check wiring.");
   hatFound = false;
   Monitor.println("HAT not found... check SDA / SCL / GND wires");
 }
 delay(2000);   // re-check every 2 seconds, forever
}
