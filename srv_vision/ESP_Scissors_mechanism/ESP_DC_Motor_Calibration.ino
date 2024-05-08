// Encoder
const int EncA = 12;
const int EncB = 13;

// PWM
const int In1 = 25;
const int In2 = 26;

int tmp = 50;

volatile bool ASet = false;
volatile long contador = 0;

void stop()
{

    analogWrite(In1, 0);
    analogWrite(In2, 0);
}

void right()
{

    analogWrite(In1, 0);
    analogWrite(In2, 30);
    delay(100);
    stop();
}

void left()
{

    analogWrite(In1, 30);
    analogWrite(In2, 0);
    delay(100);
    stop();
}

void IRAM_ATTR EncoderInterrupt()
{

    static bool ASetLast = false;

    bool ASetNow = digitalRead(EncA);

    if (ASetNow != ASetLast)
    {
        ASetLast = ASetNow;
        if (ASetNow)
        {
            contador++;
        }
    }
}

void setup()
{

    Serial.begin(9600);

    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);

    pinMode(EncA, INPUT_PULLUP);
    pinMode(EncB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(EncA), EncoderInterrupt, CHANGE);
}

void loop()
{

    if (Serial.available() > 0)
    {

        char opcion = Serial.read();

        switch (opcion)
        {

        case 'D':

            right();
            break;

        case 'I':

            left();
            break;

        case 'S':

            stop();
            break;
        }
    }

    Serial.print("Pulsos: ");
    Serial.println(contador);
}
