/*
 * yaw_with_calibration.ino
 * By: Greg Armstrong
 * July 2017
 * 
 * Calculate the yaw (heading) angle using magnetometer offset values. Before running this program
 * it is important to measure your own offset values using the calibrate_magnet program
 * 
 * This program demonstrates using the MPU9250 to calculate yaw, average a series of readings, and
 * display the result
 */

#include <Wire.h>
#include <MPU9250.h>
#include "quaternionFilters.h"

MPU9250 mpu;

int intPin = 12;

//**************************
//Magnetometer Offset Values
//**************************
//Copy the data from the serial moniter after running the calibrate_magnet program
//The offset for each axis is the average of the max and min magnetometer readings

//max readings:    21.30   869.97  2146.84 min readings:   -905.08 -14.29  1057.12
const float mxOffset = (21.30 + (-905.08)) / 2;
const float myOffset = (869.97 + (-14.29)) / 2;
const float mzOffset = (2146.84 + (1057.12)) / 2;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  initializeMPU();
}

void loop() {
    //Take an average of 10 yaw readings then print the result
    //also record how long the average took (in milliseconds) and print that as well
    int t = millis();
    float yaw = averageYaw(10);
    t =  millis() - t;
    Serial.print("delta t:\t"); Serial.print(t); Serial.print("\t");
    Serial.print("Yaw: "); Serial.println(yaw);
}

//take avgNum yaw readings and return the average
float averageYaw(int avgNum) {
    float avgYaw = 0;
    for (int i = 0; i<avgNum; i++) {
        readMPUData();
        avgYaw += mpu.yaw;
    }
    return avgYaw/((float)avgNum);
}

//Read the data from the MPU, scale the results and calculate the orientation angles (pitch, yaw, roll)
void readMPUData() {
    while(mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {}
    
    mpu.readAccelData(mpu.accelCount);
    mpu.getAres();

    mpu.ax = (float)mpu.accelCount[0] * mpu.aRes; // - accelBias[0];
    mpu.ay = (float)mpu.accelCount[1] * mpu.aRes; // - accelBias[1];
    mpu.az = (float)mpu.accelCount[2] * mpu.aRes; // - accelBias[2];

    mpu.readGyroData(mpu.gyroCount);
    mpu.getGres();

    mpu.gx = (float)mpu.gyroCount[0] * mpu.gRes;
    mpu.gy = (float)mpu.gyroCount[1] * mpu.gRes;
    mpu.gz = (float)mpu.gyroCount[2] * mpu.gRes;

    mpu.readMagData(mpu.magCount);  // Read the x/y/z adc values
    mpu.getMres();
    mpu.magbias[0] = mxOffset;
    mpu.magbias[1] = myOffset;
    mpu.magbias[2] = mzOffset;

    mpu.mx = (float)mpu.magCount[0] * mpu.mRes * mpu.magCalibration[0] - mpu.magbias[0];
    mpu.my = (float)mpu.magCount[1] * mpu.mRes * mpu.magCalibration[1] - mpu.magbias[1];
    mpu.mz = (float)mpu.magCount[2] * mpu.mRes * mpu.magCalibration[2] - mpu.magbias[2];

    mpu.updateTime();

    MahonyQuaternionUpdate(mpu.ax, mpu.ay, mpu.az, mpu.gx * DEG_TO_RAD,
                           mpu.gy * DEG_TO_RAD, mpu.gz * DEG_TO_RAD, mpu.my,
                           mpu.mx, mpu.mz, mpu.deltat);

    mpu.tempCount = mpu.readTempData();
    mpu.temperature = ((float) mpu.tempCount);

    mpu.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                              *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                      - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
    mpu.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                              *(getQ() + 2)));
    mpu.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                              *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                      - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
    mpu.pitch *= RAD_TO_DEG;
    mpu.yaw   *= RAD_TO_DEG;
    // Declination of Ithaca NY is
    //     11° 59' W (or -12°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    mpu.yaw   += 12;
    mpu.roll  *= RAD_TO_DEG;
}

//Initialize the MPU
void initializeMPU() {
  Wire.begin();

  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = mpu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71 || c == 0x73) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    mpu.MPU9250SelfTest(mpu.SelfTest);
    // Calibrate gyro and accelerometers, load biases in bias registers
    mpu.calibrateMPU9250(mpu.gyroBias, mpu.accelBias);

    mpu.initMPU9250();
    // Get magnetometer calibration from AK8963 ROM
    mpu.initAK8963(mpu.magCalibration);
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }
}

