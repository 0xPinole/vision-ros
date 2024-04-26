#include <micro_ros_arduino.h>

#include <digitalWriteFast.h>
#include <std_msgs/msg/int32.h>

#include <rclc/executor.h> 
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>

#define RIGHT_PIN 13 
#define LEFT_PIN 12 
#define PWM_PIN 15 
#define encoder_a_pin 25 
#define encoder_B_pin 23 

const int EncA = 2; 
const int EncB = 3;
const int In1 = 4;
const int In2 = 5;
const int EnA = 11; //Salida PWM 

std_msgs__msg__Int32 msg;
rcl_subscription_t subscriber; 
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support; 
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int expected_position = 0;

const float resolution = 0.0109986;

volatile int pwm_value = 50; 
volatile float position = 0; 
volatile long counter = 0;
volatile bool BSet = 0, ASet = 0;


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setup(){
  set_microros_transports(); 
  pinMode(EnA, OUTPUT);                 //Salida de PWM      
  pinMode(In1, OUTPUT);                 //Pin declarado como salida para el motor
  pinMode(In2, OUTPUT);                 //Pin declarado como salida para el motor
  pinMode(EncA, INPUT_PULLUP);          //Pin declarado como entrada, señal A del encoder de cuadratura
  pinMode(EncB, INPUT_PULLUP);          //Pin declarado como entrada, señal B del encoder de cuadratura
  attachInterrupt(0, Encoder, CHANGE);  //Leer señal A del encoder por interrupción, y asignar a Encoder 

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); 
  RCCHECK(rclc_node_init_default(&node, "scissors_motor", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "scissors-movement"));
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop(){
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  error_position = expected_position - pos; 
  pwm_value = min(max(error_position * 30, 100), 0)
  movement((error_position > 0.1), (error_position < 0.1) )
}

void movement(bool down, bool up){
  analogWrite(In1, pwm_value * down);
  analogWrite(In2, pwm_value * up); 
}

void subscription_callback(const void * msgin){
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; 
  expected_position = msg -> data; 
}


void Encoder() {
  counter += (digitalReadFast(EncB) == digitalReadFast(EncA)) || -1;
  pos = counter * resolution;
}
