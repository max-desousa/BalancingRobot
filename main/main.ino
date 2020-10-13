#include<Servo.h>
#include <PID_v1.h>
#include "quaternionFilters.h"
#include "MPU9250.h"

#define DEBUG true

#define I2CClock 400000
#define ITCPort Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

void initializeMotorsToStoppedPosition();

Servo LeftMotor;
Servo RightMotor;
MPU9250 myIMU(MPU9250_ADDRESS, ITCPort, I2CClock);

int LeftMotorPin = 7;
int RightMotorPin = 8;


//we'll use two booleans to help with reversing motor direction if they are rotating the "wrong way"
//correct the direction in SW instead of HW - I've soldered my ESC
bool ReverseLeftMotorDirection = false;
bool ReverseRightMotorDirection = false;



//As a method of guess and check, we will assume that full zero == 91 with a swing of 15 in either direction (this bidirectional ESC doesn't have the resolution I was hoping for :/ )
int StoppedSpeed = 91;
int MaxForwardSpeed = 106;
int MaxBackwardSpeed = 76;



void setup()
{
  #if DEBUG
    Serial.begin(9600);
    while(!Serial)
    {
      //Debug system not yet connected
    }
  #endif

  //we'll initialize the motors to be stopped such they won't be running out the get go.
  initializeMotorsToStoppedPosition();

  Wire.begin();

  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
}

void initializeMotorsToStoppedPosition()
{
  LeftMotor.attach(LeftMotorPin);
  RightMotor.attach(RightMotorPin);
  
  bool leftMotorInitialized = false;
  bool rightMotorInitialized = false;

  while (!leftMotorInitialized || !rightMotorInitialized)
  {
    if (LeftMotor.attached() && LeftMotor.read() == StoppedSpeed)
    {
      leftMotorInitialized = true;
    }
    else
    {
      LeftMotor.write(StoppedSpeed);
    }
    if (RightMotor.attached() && RightMotor.read() == StoppedSpeed)
    {
      rightMotorInitialized = true;
    }
    else
    {
      RightMotor.write(StoppedSpeed);
    }
    
  }
}

void loop()
{
  updateAngleData();
   
  Serial.print("Pitch: ");
  Serial.println(myIMU.pitch, 2);
}


void updateAngleData()
{
  boolean newDataFlag = true;
  if ( myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01 )
  {
    newDataFlag = true;
   
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  }//end if(myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)


  myIMU.updateTime();
    
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  myIMU.delt_t = millis() - myIMU.count;

  if (newDataFlag)
  {
    newDataFlag = false;

    myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() * *(getQ()+2)));
    myIMU.pitch *= RAD_TO_DEG;

    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  }//end if(newDataFlag)

}//end updateAngleData()
