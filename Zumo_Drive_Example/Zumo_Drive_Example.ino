/*
 * Simple Drive test
 * By: Greg Armstrong
 * July 2017
 * 
 * This program makes use of the Zumo library and TEC Bot drive functions to test the drive motors and a servo.
 * 
 * available functions
 * 
 * motors.flipLeftMotor(true)
 * motors.flipRightMotor(true)
 * reverse the direction of a motor. This has the same effect as switching the motor wires.
 * 
 * motors.setLeftSpeed(speed)
 * motors.setRightSpeed(speed)
 * Set the speed of the left and right motors. Unlike the TEC Bot functions speed is a number 
 * between -400 and 400
 * 
 * setServo(num, position)
 * Sets a servo attached to port "num" to an angle defined by position.
 * position is an integer between 0 and 180
 */

#include <ZumoMotors.h>
#include <TECBot_PWMServoDriver.h>

ZumoMotors motors;
TECBot_PWMServoDriver servos = TECBot_PWMServoDriver();


void setup() {
  Serial.begin(9600);
  servos.begin();
  servos.setPWMFreq(60);

  //Uncomment both of these lines if one or both of your motors are driving backwards
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

}

void loop() {
  for (int i = -400; i < 400; i++) {
    motors.setLeftSpeed(i);
    motors.setRightSpeed(-i);
    delay(10);
  }
  for (int i = 400; i > -400; i--) {
    motors.setLeftSpeed(i);
    motors.setRightSpeed(-i);
    delay(10);
  }
  for (int i = 0; i < 180; i++) {
    servos.setServo(4, i);
    delay(20);
  }
  for (int i = 180; i > 0; i--) {
    servos.setServo(4, i);
  }
}
