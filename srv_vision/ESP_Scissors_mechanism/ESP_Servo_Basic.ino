const int servo = 27;

int angle;
int PPM;

void setup()
{

    pinMode(servo, OUTPUT);
}

void servoPulse(int servo, int angle)
{

    // Convert angle to microseconds (PPM --- Pulse Per Minute)
    // 600 ms == 0° <---> 2400 ms == 180°

    // Reference: https://forum.arduino.cc/t/arduino-servo-libary-how-many-usec-are-1-degree/90385

    PPM = (angle * 10) + 600;

    digitalWrite(servo, HIGH);
    delayMicroseconds(PPM);

    digitalWrite(servo, LOW);

    // Refresh cycle of servo
    delay(1000);
}

void loop()
{

    for (angle = 0; angle <= 180; angle += 10)
        servoPulse(servo, angle);

    for (angle = 180; angle >= 0; angle -= 10)
        servoPulse(servo, angle);
}