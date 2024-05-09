#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h> 
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>

#include <digitalWriteFast.h>

const int EncA = 13;
const int EncB = 15;
const int In1 = 25;
const int In2 = 26;

std_msgs__msg__Int32 msg1;
std_msgs__msg__Int32 msg2;
std_msgs__msg__Int32 pub_msg;

rcl_publisher_t publisher; 
rcl_subscription_t subscriber1; 
rcl_subscription_t subscriber2; 
rclc_executor_t executor;
rclc_support_t support; 
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer; 

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("[!] RCCHECK");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("[!] RSOFTCHECK");}}

int expected_position = 0;
int camera_servo_angle = 0; 
int error_position = 0;

const float resolution = 0.000571; //0.0109986

volatile int pwm_value = 50; 
volatile int position = 0; 
volatile long counter = 0;
volatile bool direction = false;

void publisher_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg.data = position; 
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  }
}

void subscription_callback_scissors(const void * msgin){
  const std_msgs__msg__Int32 * msg1 = (const std_msgs__msg__Int32 *)msgin; 
  expected_position = msg1->data;  
}

void subscription_callback_servo(const void * msgin){
  const std_msgs__msg__Int32 * msg2 = (const std_msgs__msg__Int32 *)msgin; 
  camera_servo_angle = msg2->data; 
}

void IRAM_ATTR handleInterrupt() {
  bool r1 = digitalReadFast(EncA);
  bool r2 = digitalReadFast(EncB);
  if (r1 == r2) {
    if (direction){
      counter++;
    }else{
      counter--;
    }
  }
  position = counter * resolution;
  // needed to implement PID
}


void setup(){
  set_microros_transports(); 
  pinMode(In1, OUTPUT);                 //Pin declarado como salida para el motor
  pinMode(In2, OUTPUT);                 //Pin declarado como salida para el motor
  pinMode(EncA, INPUT_PULLUP);          //Pin declarado como entrada, señal A del encoder de cuadratura
  pinMode(EncB, INPUT_PULLUP);          //Pin declarado como entrada, señal B del encoder de cuadratura
  // attachInterrupt(digitalPinToInterrupt(EncA), handleInterrupt, CHANGE);  //Leer señal A del encoder por interrupción, y asignar a Encoder 

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); 
  RCCHECK(rclc_node_init_default(&node, "scissors_motor", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "scissors_movement"));

   RCCHECK(rclc_subscription_init_default(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "scissors_angle"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "scissors_position"));

  const unsigned int timer_timeout = 10; 
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    publisher_callback));
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg1, &subscription_callback_scissors, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg2, &subscription_callback_servo, ON_NEW_DATA)); 
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  pub_msg.data = 0;

  analogWrite(In1, 0);
  analogWrite(In2, 120);
  delay(300);
  analogWrite(In1, 120);
  analogWrite(In2, 0);
  delay(300);
  analogWrite(In1, 0);
  analogWrite(In2, 0);

  expected_position = 0; 
  position = 0;
}

void movement(bool dir){
  analogWrite(In1,  dir * pwm_value);
  analogWrite(In2, !dir * pwm_value);
}

void loop(){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
  
  error_position = expected_position - position; 
  direction = (error_position > 0);

  if(abs(error_position) > 2) {  
    movement((error_position > 0));
  } else {
    analogWrite(In1, 0);
    analogWrite(In2, 0);
  }
}