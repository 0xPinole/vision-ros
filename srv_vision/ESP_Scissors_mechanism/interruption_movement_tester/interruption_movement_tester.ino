const int EncA = 12;
const int EncB = 13;

const int In1 = 25;
const int In2 = 26;

int expected_position = 0;
int camera_servo_angle = 0;
int error_position = 0;

int pwm_value = 30;
int max_pulses = 4560;

float position = 0;
int revolutions = 0;
long current_pulse = 0;

volatile bool Aset = 0;
volatile bool Bset = 0;

bool movement_direction = true;
float last_position = 0;

int delay_timeout = 0;

void IRAM_ATTR Encoder() {
  current_pulse++;
  /*if(current_pulse == expected_position){
    pwm_value = 0;
    analogWrite(In1, 0);
    analogWrite(In2, 0);
  }*/
}

void setup(){
  Serial.begin(9600);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);


  expected_position = 0;
  position = 0;
}

void movement(bool dir, int timeout){
  analogWrite(In1,  dir * pwm_value);
  analogWrite(In2, !dir * pwm_value);
  delay(timeout);
  analogWrite(In1, 0);
  analogWrite(In2, 0);
}

void loop(){
  Serial.print(delay_timeout);
  Serial.print(" ms -> ");
  Serial.println(current_pulse);
  current_pulse = 0;
  //movement(true, 40);
  delay(4000);
  for(int i=0; i < 360; i++){
    movement(true, 30);
    delay(78);
  }

}
