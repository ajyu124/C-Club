#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

namespace robot {
namespace actuator {

class DCMotor {
 public:
  DCMotor(const uint8_t pwm_enabled_pin, 
          const uint8_t forward_pin, 
          const uint8_t backward_pin);

  void SetVelocity(const int v);
  
 private:
  uint8_t pwm_pin;
  uint8_t fwd_pin;
  uint8_t bwd_pin;
};

DCMotor::DCMotor(const uint8_t pwm_enabled_pin, 
                 const uint8_t forward_pin, 
                 const uint8_t backward_pin) {
  pwm_pin = pwm_enabled_pin;
  fwd_pin = forward_pin;
  bwd_pin = backward_pin;
  pinMode(pwm_pin, OUTPUT);
  pinMode(fwd_pin, OUTPUT);
  pinMode(bwd_pin, OUTPUT);
}

// Input must be between -255 and 255, use map() and constrain()
void DCMotor::SetVelocity(int v) {
  if (v < -255 || v > 255) {
    Serial.print(F("\nVelocity must be between -255 and 255, inclusive: "));
    Serial.print(v);
    v = constrain(v, -255, 255);
  }

  // Account for motor deadband regions where 
  // motor will not move from speeds of 0 to 80
  if (v > -80 && v < -5) v = -80;
  else if (v < 80 && v > 5) v = 80;
  
  // Write speed to PWM pin
  analogWrite(pwm_pin, abs(v));

  // Set direction to be either forward or backward, never both
  if (v > 0) {
    digitalWrite(fwd_pin, HIGH);
    digitalWrite(bwd_pin, LOW);
  } else {
    digitalWrite(bwd_pin, HIGH);
    digitalWrite(fwd_pin, LOW);
  }
}

} // namespace robot
} // namespace actuator

USB Usb;

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS3BT PS3(&Btd); // This will just create the instance

const int IN1 = 7;
const int IN2 = 6;
const int IN3 = 5;
const int IN4 = 4;

const int ENA = 9;
const int ENB = 3;

robot::actuator::DCMotor left_motor (ENB, IN3, IN4);
robot::actuator::DCMotor right_motor (ENA, IN1, IN2);

bool printAngle;

// Responsiveness of steering instead of straight motion
//const double STEER = 0.6;
const double MAX_SPEED = 255;

void setup() {

  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}


void loop() {
  Usb.Task();

  if (!PS3.PS3Connected) {
    return;
  }

  if (PS3.getButtonClick(TRIANGLE)) {
    Serial.print(F("\r\nTriangle"));
    PS3.setRumbleOn(RumbleLow);
  }
  if (PS3.getButtonClick(CIRCLE)) {
    Serial.print(F("\r\nCircle"));
    PS3.setRumbleOn(RumbleHigh);
  }

  // Left trigger
  Serial.print("\n");
  Serial.print(PS3.getAnalogButton(L2) / 255.0);
  float STEER = PS3.getAnalogButton(L2) / 255.0;

  int left_motor_v = 0;
  int right_motor_v = 0;

  // Joystick is 0 (top edge) to 255 (bottom edge)
  int joystick_vert = map(PS3.getAnalogHat(RightHatY), 
                          0, 255, 
                          (2-STEER) * floor(MAX_SPEED/2), -(2-STEER) * floor(MAX_SPEED/2));

  // Joystick is 0 (left edge) to 255 (right edge)
  int joystick_horiz = map(PS3.getAnalogHat(RightHatX), 
                           0, 255, 
                           -STEER * floor(MAX_SPEED/2), STEER * floor(MAX_SPEED/2));

  // Only move when outside of joystick deadband, which is 
  // + or - 5% around resting position
  if (joystick_vert < floor(-0.05 * MAX_SPEED/2) || 
      joystick_vert > ceil(0.05 * MAX_SPEED/2)) {
    //Serial.print(F("\tRightHatY: "));
    //Serial.print(PS3.getAnalogHat(RightHatY));
    
    right_motor_v = left_motor_v = joystick_vert;
  }
  
  // Only move when outside of joystick deadband, which is 
  // + or - 5% around resting position
  if (joystick_horiz < floor(-0.05 * MAX_SPEED/2) || 
      joystick_horiz > ceil(0.05 * MAX_SPEED/2)) {
    //Serial.print(F("\tRightHatX: "));
    //Serial.print(PS3.getAnalogHat(RightHatX));

    if (joystick_vert < -80) {
      left_motor_v -= joystick_horiz;
      right_motor_v += joystick_horiz;
    } else {
      left_motor_v += joystick_horiz;
      right_motor_v -= joystick_horiz;
    }
  }

  if (left_motor_v != 0 && right_motor_v != 0) {
    //Serial.print(F("\nMotor Velocities (L, R): "));
    //Serial.print(left_motor_v);
    //Serial.print(F(", "));
    //Serial.print(right_motor_v);
  }

  left_motor.SetVelocity(constrain(left_motor_v, -MAX_SPEED, MAX_SPEED));
  right_motor.SetVelocity(constrain(right_motor_v, -MAX_SPEED, MAX_SPEED));
}
