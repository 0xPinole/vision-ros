#include <stdio.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>


const int EncA = 12;
const int EncB = 13;

const int In1 = 25;
const int In2 = 26;

const int max_pulses = 2280;
const int pwm_value = 30;
const int move_timeout = 30;


std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 pub_msg;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("[!] RCCHECK");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("[!] RSOFTCHECK");}}

int target_motor = 0;
int current_motor = 0;
int pulses_motor = 0;
int target_pulses = 0;
bool direction_motor = true;

void reset_process(){
  if (direction_motor){
    current_motor += pulses_motor;
  }else{
    current_motor -= pulses_motor;
  }
  pulses_motor = 0;
  target_pulses = 0;
}

void start_req(){
  int error = target_motor - current_motor;
  target_pulses = abs(error);
  if(target_pulses < 27){ // 27 pulses is eq to max conversion error from 1 rad
    target_pulses = 0;
  }
  direction_motor = (error >= 0);
}

void publisher_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg.data = current_motor;
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  }
}

void subscription_callback_scissors(const void * msgin){
  reset_process();
  const std_msgs__msg__Int32 * msg_sub = (const std_msgs__msg__Int32 *)msgin;
  target_motor = msg_sub->data;
  start_req();
}

void IRAM_ATTR Encoder(){
  pulses_motor++;
}

void setup(){
  set_microros_transports();
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "scissors_motor", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "scissors_movement"));

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
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback_scissors, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  pub_msg.data = 0;
}

void movement(){
  analogWrite(In1,  direction_motor * pwm_value);
  analogWrite(In2, !direction_motor * pwm_value);
  delay(move_timeout);
  analogWrite(In1, 0);
  analogWrite(In2, 0);
}

void loop(){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(78);
  if (target_pulses > pulses_motor){
    movement();
  }else{
    reset_process();
  }
}
