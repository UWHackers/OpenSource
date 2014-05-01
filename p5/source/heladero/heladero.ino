#include "DifferentialDrive.h"

#define DPIN_YELLOW_LED 3
#define DPIN_GREEN_LED 2
#define DPIN_RED_LED 9
#define DPIN_WHITE_LED 8

#define DPIN_DRIVE_A1 7
#define DPIN_DRIVE_A2 11
#define DPIN_DRIVE_B1 6
#define DPIN_DRIVE_B2 13
#define DPIN_DRIVE_S 12
#define DPIN_DRIVE_PWMA 10
#define DPIN_DRIVE_PWMB 5

#define APIN_LIGHT_SENSOR_0 A1
#define APIN_LIGHT_SENSOR_1 A2
#define APIN_LIGHT_SENSOR_2 A3
#define APIN_LIGHT_SENSOR_3 A4
#define APIN_LIGHT_SENSOR_4 A5

#define DERIV_BUFFER_SIZE 4
#define INTEGRAL_BUFFER_SIZE 4

#define HAIRPIN_TURN_MODE

DifferentialDrive g_Drive;

float setPoint = 2.3; // Center of vehicle

#ifdef HAIRPIN_TURN_MODE
    int idleSpeed = 64;
    float kp = 360;
#else
    int idleSpeed = 196;
    float kp = 130;
#endif

float ki = 60;
float kd = 2;

// Variables used by integral and derivative control components
float derivBuffer[DERIV_BUFFER_SIZE];
float lastError;
float integralBuffer[INTEGRAL_BUFFER_SIZE];
int curDerivBufferIdx;
int curIntegralBufferIdx;

float convertSensorValueCubic(float raw_value)
{   
    // Convert to volts
    float v = (raw_value / 1023.0) * 5.0;
    
    // Cubic coefficients obtained with polyfit 
    // to empirical data in Matlab
    float a = 0.0978;
    float b = -0.5691;
    float c = 1.1800;
    float d = -0.7694;
    float adjusted_value = a*v*v*v + b*v*v + c*v + d;
    
    adjusted_value = constrain(adjusted_value, 0.0, 1.0);
    return adjusted_value;
}

float convertSensorValueLinear(float raw_value)
{   
    // Raw value will be between ~100 and ~900. We'll normalize
    // to this range. Also, there's a lot of noise from the sensor,
    // so any values under 400 will be capped to 0
    float adjusted_value;
    raw_value = constrain(raw_value, 100, 800);
    if (raw_value < 400)
    {
        adjusted_value = (raw_value / 400) * 0.10;
    }
    else if (raw_value < 700)
    {
         adjusted_value = 0.10 + ((raw_value - 400.0) / 300.0) * 0.40;
    }
    else
    {
        adjusted_value  = 0.5 + ((raw_value - 700.0) / 100.0) * 0.5;
    }
    adjusted_value = constrain(adjusted_value, 0.0, 1.0);
    return adjusted_value;
}

void clearLeds()
{
    digitalWrite(DPIN_WHITE_LED, LOW);
    digitalWrite(DPIN_RED_LED, LOW);
    digitalWrite(DPIN_GREEN_LED, LOW);
    digitalWrite(DPIN_YELLOW_LED, LOW);
}

// Give a representation of the current line position using LEDs
void setLedsFromLinePos(float linePos)
{
    clearLeds();
    
    if ((linePos >= 0.3) && (linePos < 1.3))
    {
        // Set white LED
        digitalWrite(DPIN_WHITE_LED, HIGH);
    }
    else if ((linePos >= 1.3) && (linePos < 2.3))
    {
        // Set red LED
        digitalWrite(DPIN_RED_LED, HIGH);
    }
    else if ((linePos >= 2.3) && (linePos < 3.3))
    {
        // Set green LED
        digitalWrite(DPIN_GREEN_LED, HIGH);
    }    
    else if ((linePos >= 3.3) && (linePos < 4.3))
    {
        // Set red LED
        digitalWrite(DPIN_YELLOW_LED, HIGH);
    }
    else
    {
        // Can't see line; turn on all LEDs
        digitalWrite(DPIN_WHITE_LED, HIGH);
        digitalWrite(DPIN_RED_LED, HIGH);
        digitalWrite(DPIN_GREEN_LED, HIGH);
        digitalWrite(DPIN_YELLOW_LED, HIGH);        
    }
}

float convertSensorValue(float data)
{
    return convertSensorValueLinear(data);
    // return convertSensorValueCubic(data);    
    
}

float getLinePosFromSensorData(int raw_data[])
{   
    // First, convert raw sensor values to a number between 0 and 1
    float s0 = convertSensorValue((float)raw_data[0]);
    float s1 = convertSensorValue((float)raw_data[1]);
    float s2 = convertSensorValue((float)raw_data[2]);
    float s3 = convertSensorValue((float)raw_data[3]);
    float s4 = convertSensorValue((float)raw_data[4]);
    
    // Each sensor has a position in centimeters. We use these positions as
    // weights, multiplied by whether there seems to be a line or not, and then
    // summed and divided by number of non-zero values to get an estimate of the
    // actual line position.
    float weightedSensorSum = s0 * 0.3 + s1 * 1.3 + s2 * 2.3 + s3 * 3.3 + s4 * 4.3;
    float unweightedSensorSum = s0 + s1 + s2 + s3 + s4;
    float linePos;
    if (unweightedSensorSum < 0.35)
    {
        // There's no line!
        linePos = 2.3;
    }
    else
    {
        linePos = weightedSensorSum / unweightedSensorSum;
    }
    
    return linePos;
}

// Implement P controler
float PController(float err)
{
    // Proportional component
    
    float p = kp * err;
     
    return p;
}

// Implement PID controller
float PIDController(float err)
{
    // Proportional component
    
    float p = kp * err;
    
    // Integral component

    // Insert current value into integral buffer for calculation
    integralBuffer[curIntegralBufferIdx] = err;
    curIntegralBufferIdx++;
    if (curIntegralBufferIdx >= INTEGRAL_BUFFER_SIZE)
    {
        curIntegralBufferIdx = 0;    
    }
    
    // Calculate error integral
    float integralError = 0.0;
    for (int i = 0; i < INTEGRAL_BUFFER_SIZE; i++)
    {
        integralError += integralBuffer[i];
    }
    float i = ki * integralError / INTEGRAL_BUFFER_SIZE; // Normalized
    
    // Derivative component
    
    // Calculate current error derivative and store in buffer
    float derivErr = err - lastError;
    derivBuffer[curDerivBufferIdx] = derivErr;
    curDerivBufferIdx++;
    if (curDerivBufferIdx >= DERIV_BUFFER_SIZE)
    {
        curDerivBufferIdx = 0;    
    }

    // Add to buffer of derivative errors and take the average
    float derivSum = 0.0;
    for (int i = 0; i < DERIV_BUFFER_SIZE; i++)
    {
        derivSum += derivBuffer[i];
    }
    float derivAvg = derivSum / DERIV_BUFFER_SIZE;

    float d = kd * derivAvg;
    
    return (p + i + d);
}

void convertToWheelSpeeds(float controllerOutput, int* wheelSpeeds)
{
    // Negative controller output means turn right, s we'll add to the left wheel speed
    if (controllerOutput < 0)
    {
        wheelSpeeds[1] = idleSpeed;        
        wheelSpeeds[0] = idleSpeed + floor(abs(controllerOutput));
        
        // Adjust right wheel speed to compensate if left wheel actuator is saturated
        if (wheelSpeeds[0] > 255)
        {
            float saturationAmount = wheelSpeeds[0] - 255;
            wheelSpeeds[1] = wheelSpeeds[1] - saturationAmount;
            wheelSpeeds[0] = 255;
        }
    }
    else
    {
        wheelSpeeds[1] = idleSpeed + floor(abs(controllerOutput));
        wheelSpeeds[0] = idleSpeed;        
 
        // Adjust left wheel speed to compensate if right wheel actuator is saturated
        if (wheelSpeeds[1] > 255)
        {
            float saturationAmount = wheelSpeeds[1] - 255;
            wheelSpeeds[0] = wheelSpeeds[0] - saturationAmount;
            wheelSpeeds[1] = 255;
        }   
    }
}

void setup()
{
    Serial.begin(9600);
    
    g_Drive.SetOutputPins(
     DPIN_DRIVE_A1,
     DPIN_DRIVE_A2,
     DPIN_DRIVE_PWMA,
     DPIN_DRIVE_B1,
     DPIN_DRIVE_B2,
     DPIN_DRIVE_PWMB,
     DPIN_DRIVE_S);

    // Initialize arrays that will be used for derivative
    // and integral components
    for (int i = 0; i < DERIV_BUFFER_SIZE; i++)
    {
        derivBuffer[i] = 0.0;
        
    }
    for (int i = 0; i < INTEGRAL_BUFFER_SIZE; i++)
    {
        integralBuffer[i] = 0;
    }
    
    lastError = 0.0;
    curDerivBufferIdx = 0;
    curIntegralBufferIdx = 0;

}

void loop()
{  
    int lightSensors[5];
    lightSensors[0] = analogRead(APIN_LIGHT_SENSOR_0);
    lightSensors[1] = analogRead(APIN_LIGHT_SENSOR_1);
    lightSensors[2] = analogRead(APIN_LIGHT_SENSOR_2);
    lightSensors[3] = analogRead(APIN_LIGHT_SENSOR_3);
    lightSensors[4] = analogRead(APIN_LIGHT_SENSOR_4);    
    
    float linePos = getLinePosFromSensorData(lightSensors);

    setLedsFromLinePos(linePos);
    
    // Get error for this timestep
    float err = setPoint - linePos;    

    // Output is value that the controller wants the motors to turn;
    // if negative, turn left; if positive, turn right
    float controllerOutput = PIDController(err);
    
    int wheelSpeeds[2];
    convertToWheelSpeeds(controllerOutput, wheelSpeeds);
    
    g_Drive.setWheelSpeeds(wheelSpeeds[0], wheelSpeeds[1]);
}
