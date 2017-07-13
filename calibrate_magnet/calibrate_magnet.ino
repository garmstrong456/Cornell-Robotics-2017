/*
 * calibrate_magnet.ino
 * By: Greg Armstrong
 * July 2017
 * 
 * Before you can use the magnetometer as a compass you must compensate for the fixed magnetic field present
 * on your robot.
 * 
 * This measures the magnetometer offset values. Run this program and open the serial moniter. Slowly rotate your
 * robot around each of its three axes several times. The program automatically keeps track of the max and min
 * magnetic field values measured. After moving the robot around copy and paste the max and min values from the
 * serial monitor to the sketch where you want to calculate heading or orientation angles.
 */

#include <Wire.h>
#include <MPU9250.h>

int intPin = 12;

MPU9250 mpu;

//initalize the max values to the lowest float and the min values to the highest
float maxMx = -3.4028235E+38, maxMy = -3.4028235E+38, maxMz = -3.4028235E+38;
float minMx =  3.4028235E+38, minMy =  3.4028235E+38, minMz =  3.4028235E+38;

void setup() {
  Serial.begin(9600);
  while (!Serial);

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

void loop() {
  //Wait for new data to be available
  if (mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    
    //Read and scale the magnetometer values
    mpu.readMagData(mpu.magCount);
    mpu.getMres();
    mpu.mx = (float)mpu.magCount[0] * mpu.mRes * mpu.magCalibration[0] - mpu.magbias[0];
    mpu.my = (float)mpu.magCount[1] * mpu.mRes * mpu.magCalibration[1] - mpu.magbias[1];
    mpu.mz = (float)mpu.magCount[2] * mpu.mRes * mpu.magCalibration[2] - mpu.magbias[2];

    //check to see if any values are larger than the current max
    if (mpu.mx > maxMx) maxMx = mpu.mx;
    if (mpu.my > maxMy) maxMy = mpu.my;
    if (mpu.mz > maxMz) maxMz = mpu.mz;

    //check to see if any values are smaller than the current min
    if (mpu.mx < minMx) minMx = mpu.mx;
    if (mpu.my < minMy) minMy = mpu.my;
    if (mpu.mz < minMz) minMz = mpu.mz;

    //display the results
    Serial.print("current readings:\t");
    Serial.print(mpu.mx); Serial.print("\t");
    Serial.print(mpu.my); Serial.print("\t");
    Serial.print(mpu.mz); Serial.print("\t");

    Serial.print("max readings:\t");
    Serial.print(maxMx); Serial.print("\t");
    Serial.print(maxMy); Serial.print("\t");
    Serial.print(maxMz); Serial.print("\t");

    Serial.print("min readings:\t");
    Serial.print(minMx); Serial.print("\t");
    Serial.print(minMy); Serial.print("\t");
    Serial.print(minMz); Serial.print("\t");
    Serial.println();
  }
}
