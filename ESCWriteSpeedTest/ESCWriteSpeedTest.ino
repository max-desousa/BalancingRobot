#include <Servo.h>

#define rightServoPin 7
#define leftServoPin 8

#define ESCStoppedPosition 91
#define ESCMaxSpeedPosition 106

Servo leftMotor;
Servo rightMotor;

void setup() {
  // put your setup code here, to run once:

  bool leftMotorIsGood = false;
  bool rightMotorIsGood = false;

  while(leftMotorIsGood != true && rightMotorIsGood != true)
  {
    if(leftMotor.attached() == false)
    {
      leftMotor.attach(leftServoPin);
    }
    if(rightMotor.attached() == false)
    {
      rightMotor.attach(rightServoPin);
    }
    
    if(leftMotor.attached() == true && leftMotor.read() != ESCStoppedPosition)
    {
      leftMotor.write(ESCStoppedPosition);
    }
    if(rightMotor.attached() == true && rightMotor.read() != ESCStoppedPosition)
    {
      rightMotor.write(ESCStoppedPosition);
    }

    if(leftMotor.attached() == true && leftMotor.read() == ESCStoppedPosition)
    {
      leftMotorIsGood = true;
    }
    if(rightMotor.attached() == true && rightMotor.read() == ESCStoppedPosition)
    {
      leftMotorIsGood = true;
    }
  }
  
  unsigned long startTime = millis();

  while( millis() < (startTime + 5000) )
  {
    
  }
  
}//void setup()

void loop() {
  // put your main code here, to run repeatedly:

  leftMotor.write(ESCMaxSpeedPosition);
  rightMotor.write(ESCMaxSpeedPosition);

}
