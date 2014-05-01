
#include "DifferentialDrive.h"

DifferentialDrive::DifferentialDrive()
{

}

void DifferentialDrive::SetOutputPins( int a1, int a2, int pwmA, int b1, int b2, int pwmB, int sdby )
{
  m_PinA1 = a1;
  m_PinA2 = a2;
  m_PinPwmA = pwmA;
  m_PinB1 = b1;
  m_PinB2 = b2;
  m_PinPwmB = pwmB;
  m_PinStandby = sdby;

  pinMode(m_PinA1, OUTPUT);
  pinMode(m_PinA2, OUTPUT);
  pinMode(m_PinPwmA, OUTPUT);
  pinMode(m_PinB1, OUTPUT);
  pinMode(m_PinB2, OUTPUT);
  pinMode(m_PinPwmB, OUTPUT);
  pinMode(m_PinStandby, OUTPUT);

  Stop();
}

void DifferentialDrive::SetPhysicalDimensions(float wheelRadius, float wheelOffset)
{
  m_flWheelRadius = wheelRadius;
  m_flWheelOffset = wheelOffset;
}

void DifferentialDrive::Stop()
{
  digitalWrite(m_PinStandby, LOW);
  digitalWrite(m_PinA1, LOW);
  digitalWrite(m_PinA2, LOW);
  digitalWrite(m_PinB1, LOW);
  digitalWrite(m_PinB2, LOW);

  analogWrite(m_PinPwmA, 0);
  analogWrite(m_PinPwmB, 0);
}

void DifferentialDrive::DriveManual( byte a1, byte a2, byte pwmA, byte b1, byte b2, byte pwmB, byte s)
{
  a1 = (a1 > 0) ? HIGH : LOW;
  a2 = (a2 > 0) ? HIGH : LOW;
  b1 = (b1 > 0) ? HIGH : LOW;
  b2 = (b2 > 0) ? HIGH : LOW;
  s = (s > 0) ? HIGH : LOW;

  digitalWrite(m_PinStandby, s);
  digitalWrite(m_PinA1, a1);
  digitalWrite(m_PinA2, a2);
  digitalWrite(m_PinB1, b1);
  digitalWrite(m_PinB2, b2);

  analogWrite(m_PinPwmA, pwmA);
  analogWrite(m_PinPwmB, pwmB);
}

void DifferentialDrive::Drive( float v, float w )
{
  //int iV = lrint(trunc(v))
  //doTurn( ;
}

void DifferentialDrive::Pivot( int iRate )
{
  if (iRate == 0)
  {
    digitalWrite(m_PinStandby, LOW);
  }
  else
  {
    digitalWrite(m_PinStandby, HIGH);
    doTurn(iRate, -iRate);
  }
}

void DifferentialDrive::setWheelSpeeds(int speedLeft, int speedRight)
{
  
  digitalWrite(m_PinStandby, HIGH);
  setDirectionPins( speedLeft, speedRight );
  
  speedRight = abs(speedRight);
  if (speedRight > 255)
    speedRight = 255;
    
  speedLeft = abs(speedLeft);
  if (speedLeft > 255)
    speedLeft = 255;

  analogWrite(m_PinPwmA, speedRight);
  analogWrite(m_PinPwmB, speedLeft);
}

void DifferentialDrive::doTurn(int iSpeedLeft, int iSpeedRight)
{
  iSpeedLeft += 128;
  iSpeedRight += 128;
  
  digitalWrite(m_PinStandby, HIGH);
  setDirectionPins( iSpeedLeft, iSpeedRight );
  
  iSpeedRight = abs(iSpeedRight);
  if (iSpeedRight > 255)
    iSpeedRight = 255;
    
  iSpeedLeft = abs(iSpeedLeft);
  if (iSpeedLeft > 255)
    iSpeedLeft = 255;

  analogWrite(m_PinPwmA, iSpeedRight);
  analogWrite(m_PinPwmB, iSpeedLeft);
}

void DifferentialDrive::setDirectionPins(int iSpeedLeft, int iSpeedRight)
{
  if (iSpeedRight < 0)
  {
    digitalWrite(m_PinA1, LOW);
    digitalWrite(m_PinA2, HIGH);
  }
  else
  {
    digitalWrite(m_PinA2, LOW);
    digitalWrite(m_PinA1, HIGH);
  }

  if (iSpeedLeft < 0)
  {
    digitalWrite(m_PinB1, LOW);
    digitalWrite(m_PinB2, HIGH);
  }
  else
  {
    digitalWrite(m_PinB2, LOW);
    digitalWrite(m_PinB1, HIGH);
  } 
}




