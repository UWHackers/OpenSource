
#ifndef __H_DIFFERENTIAL_DRIVE_
#define __H_DIFFERENTIAL_DRIVE_

#include "Arduino.h"

class DifferentialDrive
{
public:
  DifferentialDrive();

  void SetPhysicalDimensions(float wheelRadius, float wheelOffset);
  void SetOutputPins( int a1, int a2, int pwmA, int b1, int b2, int pwmB, int sdby );

  void Stop();

  void Drive( float v, float w );
  
  void Pivot( int iRate );

  void DriveManual( byte a1, byte a2, byte pwmA, byte b1, byte b2, byte pwmB, byte s);

  //private:
  void doTurn(int iSpeedLeft, int iSpeedRight);
  void setWheelSpeeds(int speedLeft, int speedRight);
  void setDirectionPins(int iSpeedLeft, int iSpeedRight);

private:
  int m_PinA1;
  int m_PinA2;
  int m_PinPwmA;
  int m_PinB1;
  int m_PinB2;
  int m_PinPwmB;
  int m_PinStandby;

  float m_flWheelRadius;
  float m_flWheelOffset;
};

#endif // #ifndef __H_DIFFERENTIAL_DRIVE_

