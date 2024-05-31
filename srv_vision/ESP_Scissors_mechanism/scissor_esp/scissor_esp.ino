#include <stdio.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <geometry_msgs/msg/pose2_d.h>

const int servo_01 = 25;

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

int current_servo_01 = 0;
int target_servo_01 = 0;

int angle;
int ppm;

void publisher_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg.y = 36;
    pub_msg.x = 0;
    pub_msg.theta = current_servo_01;

    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  }
}

void subscription_callback_scissors(const void * msgin){
  const geometry_msgs__msg__Pose2D * msg_sub = (const geometry_msgs__msg__Pose2D *)msgin;
  // target_motor = (int)msg_sub->y;
  target_servo_01 = (int)msg_sub->theta;
}

void servoPulse(int servoPin){
  ppm = map(angle, 0, 180, 500, 2500);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(ppm);
  digitalWrite(servoPin, LOW);
}

void setup(){
  set_microros_transports();

  pinMode(servo_01, OUTPUT);

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
    "scissors_movement_pub"));

  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    publisher_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback_scissors, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop(){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  if(target_servo_01 != current_servo_01){
    angle = current_servo_01 + ((target_servo_01 > current_servo_01) * 2)-1;
    servoPulse(servo_01);
    current_servo_01 = angle;
  }
}
