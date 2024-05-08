// Encoder
const int EncA = 12;
const int EncB = 13;

// PWM
const int In1 = 25;
const int In2 = 26;

// Interruptions var
volatile bool Aset = 0;
volatile bool Bset = 0;

// Motor var
int ppr = 4560;       // Pulses per revolution
float enc_res = 0.12; // Resolucion del encoder
volatile int n = 0;   // Current number of pulses
int motor_pos = 0;    // Motor position in degree's
int pwm = 30;         // (30 - 250) min and max values to move the motor

// Others
long current_pulse = 0 : long current_pulse_aux = 0;
long revolutions = 0;
float conversion = 0;

float pos = 0;
float prev_pos = 0;
float prev_pos = 0;
float current_pos = 0;

void stop()
{

    analogWrite(In1, 0);
    analogWrite(In2, 0);
}

void turn_rigth(int pwm)
{

    analogWrite(In1, 0);
    analogWrite(In2, pwm);
}

void turn_left(int pwm)
{

    analogWrite(In1, pwm);
    analogWrite(In2, 0);
}

// Get motor direction
void IRAM_ATTR Encoder()
{

    Aset = digitalRead(EncA);
    Bset = digitalRead(EncB);

    // Left
    if (BSet == ASet)
    {

        current_pulse++;
        current_pulse_aux++;

        // Transform pulse to degrees
        pos = current_pulse * enc_res;
        current_pos = current_pulse_aux * enc_res;

        // Reset pulses per revolution
        if (current_pulse >= ppr)
        {
            revolutions++;
            current_pulse = 0;
        }
    }

    // Rigth
    else
    {

        current_pulse--;
        current_pulse_aux--;

        // Transform pulse to degrees
        pos = current_pulse * enc_res;
        current_pos = current_pulse_aux * enc_res;

        // Reset pulses per revolution
        if (contador <= -pulsos)
        {

            revoluciones--;
            contador = 0;
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

    attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncB), Encoder, CHANGE);
}

void loop()
{

    if (milis() - lastTime >= sampleTime || lastTime == 0)
    {

        // Update lastime
        lastTime = millis();

        // Set position in degrees
        motor_pos = (n * 360.0) / enc;

        Serial.print("Posicion en grados: ");
        Serial.println(P);
    }

    if (Serial.available() > 0)
    {

        int target = Serial.read();

        if (target > 0)
        {

            turn_rigth();
        }
        else
        {

            turn_left();
        }
    }
}
