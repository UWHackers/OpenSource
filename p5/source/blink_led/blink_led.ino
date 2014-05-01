int ledPinNumber = 3; // LED connected to digital pin 3
int period = 500; // LED on for 500 ms, off for 500ms

void setup()
{
    // Set the LED pin to output mode
    pinMode(ledPinNumber, OUTPUT);
}

void loop()
{
    // Turn on LED
    digitalWrite(ledPinNumber, HIGH);
    // Wait
    delay(period);
    // Turn off LED
    digitalWrite(ledPinNumber, LOW);
    // Wait
    delay(period);
}
