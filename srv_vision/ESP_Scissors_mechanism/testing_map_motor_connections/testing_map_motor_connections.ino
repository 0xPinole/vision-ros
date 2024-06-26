// Encoder
const int EncA = 12;
const int EncB = 13;

// PWM
const int In1 = 25;
const int In2 = 26;

// Interruptions variables
volatile bool Aset = 0;
volatile bool Bset = 0;

// Motor variables
int ppr = 4560;       // Pulses per revolution
float enc_res = 0.12; // Encoder resolution
int motor_pos = 0;    // Motor position in degrees
int pwm = 30;         // PWM value (30 - 250) for motor movement

// Others
hw_timer_t *timer = NULL;

long current_pulse = 0;
long revolutions = 0;
float prev_pos = 0;
float current_pos = 0;
float linear_vel = 0.0;

void reset()
{

    current_pulse = 0;
    revolutions = 0;
    linear_vel = 0.0;
}

void stop()
{

    analogWrite(In1, 0);
    analogWrite(In2, 0);
}

void turn_right(int pwm)
{

    analogWrite(In1, pwm);
    analogWrite(In2, 0);
}

void turn_left(int pwm)
{

    analogWrite(In1, 0);
    analogWrite(In2, pwm);
}

// Encoder interruption routine
void IRAM_ATTR Encoder()
{

    Aset = digitalRead(EncA);
    Bset = digitalRead(EncB);

    if (Bset == Aset)
    {

        current_pulse++;
    }
    else
    {

        current_pulse--;
    }

    motor_pos = (current_pulse * 360) / ppr;
    current_pos = motor_pos;

    if (current_pulse >= ppr || current_pulse <= -ppr)
    {
        revolutions++;
        current_pulse = 0;
    }
}

// Velocity calculation interruption routine
void IRAM_ATTR getVel()
{

    linear_vel = (current_pos - prev_pos) / 0.015;

    if (linear_vel < 0)
        linear_vel = -linear_vel;

    prev_pos = current_pos;
}

void reachTarget(char direction, int distance)
{

    int initial_pulse = current_pulse;
    int pulses_needed = distance / enc_res;

    if (direction == 'L')
    {

        turn_left(pwm);
    }
    else if (direction == 'R')
    {

        turn_right(pwm);
    }

    while (abs(current_pulse - initial_pulse) < abs(pulses_needed))
    {

        Serial.print("Velocity: ");
        Serial.println(linear_vel);

        Serial.print("Direction: ");
        Serial.println(direction);

        float distance_covered = abs(current_pulse - initial_pulse) * enc_res;
        Serial.print("Covered distance: ");
        Serial.print(distance_covered);
        Serial.println(" cm");
    }

    stop();
    reset();
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

    timer = timerBegin(0, 80, true); // Timer 0, prescaler 80
    timerAttachInterrupt(timer, &getVel, true);
    timerAlarmWrite(timer, 15000, true); // 0.015 s interval
    timerAlarmEnable(timer);
}

void loop()
{

    if (Serial.available() > 0)
    {

        String str = Serial.readStringUntil(' ');

        char direction = str.charAt(0);

        str.remove(0, 1);
        int distance = str.toInt();

        reachTarget(direction, distance);
    }
}
