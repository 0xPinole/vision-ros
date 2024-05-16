#include <stdio.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <geometry_msgs/msg/pose2_d.h>


const int EncA = 12;
const int EncB = 13;

const int In1 = 25;
const int In2 = 26;

const int servo = 27;

const int max_pulses = 2280;
const int pwm_value = 30;
const int move_timeout = 30;

geometry_msgs__msg__Pose2D pub_msg;
geometry_msgs__msg__Pose2D msg;

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

int current_servo = 0;
int target_servo = 0;

void reset_process(){
  if (direction_motor){
    current_motor += pulses_motor;
  }else{
    current_motor -= pulses_motor;
  }
  pulses_motor = 0;
  target_pulses = 0;
}

void publisher_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg.y = current_motor;
    pub_msg.x = 0;
    pub_msg.theta = current_servo;

    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  }
}

void subscription_callback_scissors(const void * msgin){
  const geometry_msgs__msg__Pose2D * msg_sub = (const geometry_msgs__msg__Pose2D *)msgin;
  target_motor = (int)msg_sub->y;
  int error = target_motor - current_motor;
  target_pulses = abs(error);
  if(target_pulses < 27){ // 27 pulses is eq to max conversion error from 1 rad
    target_pulses = 0;
  }
  direction_motor = (error >= 0);

  target_servo = (int)msg_sub->theta;
}

void IRAM_ATTR Encoder(){
  pulses_motor++;
}

void servoPulse(int angle)
{

    // Convert angle to microseconds (PPM --- Pulse Per Minute)
    // 600 ms == 0° <---> 2400 ms == 180°
    // Reference: https://forum.arduino.cc/t/arduino-servo-libary-how-many-usec-are-1-degree/90385

    int PPM = (angle * 10) + 600;

    digitalWrite(servo, HIGH);
    delayMicroseconds(PPM);

    digitalWrite(servo, LOW);

    // Refresh cycle of servo
    delay(1000);
}

void setup(){
  set_microros_transports();

  pinMode(servo, OUTPUT);

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "scissors_mechanism", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
    "scissors_movement"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
    "scissor_movement_pub"));

  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    publisher_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback_scissors, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
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
  delay(80);
  if (target_pulses > pulses_motor){
    movement();
  }else{
    reset_process();
    if(target_servo != current_servo){
      servoPulse(target_servo);
      current_servo = target_servo;
    }
  }
}
