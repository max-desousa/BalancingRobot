#include<Servo.h>

#define DEBUG true

void initializeMotorsToStoppedPosition();

Servo LeftMotor;
Servo RightMotor;

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

unsigned long SetupStartedTime = 0;
unsigned long SetupFinishedTime = 0;
bool OneShotTracker = false;

void setup()
{
  #if DEBUG
    Serial.begin(9600);
    while(!Serial)
    {
      //Debug system not yet connected
    }
  #endif
  SetupStartedTime = micros();
  LeftMotor.attach(LeftMotorPin);
  RightMotor.attach(RightMotorPin);

  //we'll initialize the motors to be stopped such they won't be running out the get go.
  initializeMotorsToStoppedPosition();
}

void initializeMotorsToStoppedPosition()
{
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
   
}
