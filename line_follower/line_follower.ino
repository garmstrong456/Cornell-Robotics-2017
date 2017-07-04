/*
 * Demo line-following code for the TEC-Bot 4.2
 * By: Greg Armstrong
 * July 2017
 * 
 * Uses the Pololu QTR-3RC Reflectance Sensor array to detect a black line
 * on a white background. Electrical tape on a white surface works well
 *
 * http://www.pololu.com/catalog/product/2457
 */

#include <TECBot_PWMServoDriver.h>
#include <QTRSensors.h>

#define SERIAL_DEBUG true

//Initialize the line sensor variables
#define TIMEOUT 2500
QTRSensorsRC lineSensor((unsigned char[]) {4, 5, 6}, 3, TIMEOUT, QTR_NO_EMITTER_PIN);

//Define the max speed allowed
//Changing this will probably require you to tweak the PID constants
#define MAX_SPEED 20
int lastError = 0;

TECBot_PWMServoDriver drive = TECBot_PWMServoDriver();


void setup() {
    Serial.begin(9600);

    //Initialize the PWM Driver board
    drive.begin();
    drive.setPWMFreq(60);

    //Arm the ESC's
    drive.setDrive(0,0);
    delay(1500);

    //Calibrate the sensor
    Serial.println("Calibrating");
    digitalWrite(13, HIGH);         //Turn the onboard LED on while calibrating
    delay(1000);

    //Turn back and forth over the line twice
    for (int i; i < 80; i++) {
        if ((i > 10 && i <= 30) || (i > 50 && i < 70)) {
            drive.setDrive(5, -5);
        } else {
            drive.setDrive(-5, 5);
        }
        lineSensor.calibrate();
        delay(20);
    }
    Serial.println("Done");
    digitalWrite(13, LOW);          //Turn the LED off
}

void loop() {
    //Read the line position from the sensor
    unsigned int sensors[3];
    int position = lineSensor.readLine(sensors);

    //Calculate the error
    int error = position - 1000;

    //Calculate the required correction
    int speedDifference = error/50 + 1*(error - lastError);

    //record the error in last error
    lastError = error;

    //Calculate the motor speeds
    int leftSpeed = MAX_SPEED - speedDifference;
    int rightSpeed = MAX_SPEED + speedDifference;

    //Make sure the motor speeds stay between 0 and MAX_SPEED
    //For some applications you might want to reduce the lower bound
    //and allow the motors to turn backwards
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

    //Set the drive motors
    drive.setDrive(-leftSpeed, -rightSpeed);

    //Dump a bunch of information to the Serial monitor if debug is enabled
    if (SERIAL_DEBUG) {
        Serial.print("sensor: ");
        Serial.print(sensors[0]); Serial.print("\t");
        Serial.print(sensors[1]); Serial.print("\t");
        Serial.print(sensors[2]); Serial.print("\t");
        Serial.print(position); Serial.print("\t");
        Serial.print("error: ");
        Serial.print(error); Serial.print("\t");
        Serial.print(speedDifference); Serial.print("\t");
        Serial.print("motor: ");
        Serial.print(leftSpeed); Serial.print("\t");
        Serial.print(rightSpeed); Serial.println();
    }
}
