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

DifferentialDrive g_Drive;
// float sensorThreshold = 500.0;
float setPoint = 2.3; // Center of vehicle
int idleSpeed = 200;

// Ku = 300
// Tu = 25s, or 1388 frames
float kp = 130;
float ki = 30;
float kd = 2;

// Variables used by integral and derivative control components
float derivBuffer[DERIV_BUFFER_SIZE];
float lastError;
float integralBuffer[INTEGRAL_BUFFER_SIZE];
int curDerivBufferIdx;
int curIntegralBufferIdx;


float convertSensorValue(float raw_value)
{
//    if (raw_value > sensorThreshold)
//    {
//        return 1.0;
//    }
//    else
//    {
//        return 0.0;
//    }
//    
//    return constrain((raw_value - 300.0) / 600.0, 0.0, 1.0);
    
    

    // Raw value will be between ~100 and ~900. We'll normalize
    // to this range. Also, there's a lot of noise from the sensor,
    // so any values under 400 will be capped to 0
    float adjusted_value;
    raw_value = constrain(raw_value, 100, 800);
    if (raw_value < 400)
    {
        adjusted_value = (raw_value / 400) * 0.13;
    }
    else if (raw_value < 700)
    {
         adjusted_value = 0.13 + ((raw_value - 400.0) / 300.0) * 0.37;
    }
    else
    {
        adjusted_value  = 0.15 + ((raw_value - 700.0) / 100.0) * 0.5;
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

float getLinePosFromSensorData(int raw_data[])
{
//    Serial.println("Raw data");
//    Serial.print(raw_data[0]);
//    Serial.print(" ");
//    Serial.print(raw_data[1]);
//    Serial.print(" ");
//    Serial.print(raw_data[2]);
//    Serial.print(" ");
//    Serial.print(raw_data[3]);
//    Serial.print(" ");
//    Serial.println(raw_data[4]);
////    
    
    // First, convert raw sensor values to a number between 0 and 1
    float s0 = convertSensorValue(raw_data[0]);
    float s1 = convertSensorValue(raw_data[1]);
    float s2 = convertSensorValue(raw_data[2]);
    float s3 = convertSensorValue(raw_data[3]);
    float s4 = convertSensorValue(raw_data[4]);

//    Serial.println("Adjusted values:");
//    Serial.print(s0);
//    Serial.print(" ");
//    Serial.print(s1);
//    Serial.print(" ");
//    Serial.print(s2);
//    Serial.print(" ");
//    Serial.print(s3);
//    Serial.print(" ");
//    Serial.println(s4);
//    Serial.println("");
    
    // Each sensor has a position in centimeters. We use these positions as
    // weights, multiplied by whether there seems to be a line or not, and then
    // summed and divided by number of non-zero values to get an estimate of the
    // actual line position.
    float weightedSensorSum = s0 * 0.3 + s1 * 1.3 + s2 * 2.3 + s3 * 3.3 + s4 * 4.3;
    float unweightedSensorSum = s0 + s1 + s2 + s3 + s4;
    float linePos;
    if (unweightedSensorSum == 0)
    {
        // There's no line!
        linePos = 0.0;
    }
    else
    {
        linePos = weightedSensorSum / unweightedSensorSum;
    }
    
//    Serial.print("linePos: ");
//    Serial.println(linePos);
//    Serial.println("");
    
    return linePos;
}

// Implement P controler
float PController(float err)
{
    // Proportional component
    
    float p = kp * err;
     
    return p;
}

// Implement PID controler
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
    
//    Serial.print("p: ");
//    Serial.println(p);
//    Serial.print("i: ");
//    Serial.println(i);
//    Serial.print("d: ");
//    Serial.println(d);
//    Serial.print("Sum: ");
//    Serial.println(p + i + d);
//    Serial.println("=========");    
    
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
//    Serial.print("Left wheel: ");
//    Serial.println(wheelSpeeds[0]);
//    Serial.print("Right wheel: ");
//    Serial.println(wheelSpeeds[1]);
//    Serial.println("");
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

     // delay(3000);

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
    
    // Set gains (for debugging)
//    float ku = 70.0;
//    kp = 0.6 * ku;
//    ki = (2.0 * kp) / PERIOD;
//    kd = (kp * PERIOD) / 8;

//    Serial.print("kp: ");
//    Serial.println(kp);
//    Serial.print("ki: ");
//    Serial.println(ki);
//    Serial.print("kd: ");
//    Serial.println(kd);    

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
//    Serial.print("Line position: ");
//    Serial.println(linePos);
    setLedsFromLinePos(linePos);
    
    // Get error for this timestep
    float err = setPoint - linePos;    

    // Output is value that the controller wants the motors to turn;
    // if negative, turn left; if positive, turn right
    float controllerOutput = PIDController(err);
    
    int wheelSpeeds[2];
    convertToWheelSpeeds(controllerOutput, wheelSpeeds);
    
    g_Drive.setWheelSpeeds(wheelSpeeds[0], wheelSpeeds[1]);
    
//    Serial.print("Left wheel: ");
//    Serial.print(wheelSpeeds[0]);
//    Serial.print("   Right wheel: ");
//    Serial.println(wheelSpeeds[1]);
}
