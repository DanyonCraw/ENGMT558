#include <ESP32Servo.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#define SERVO_PIN 33
#define LEDPIN_R 14
#define LEDPIN_G 27
#define LEDPIN_B 26
#define BUTTON_PIN 25

Servo servo;

rcl_publisher_t publisher_int;
rcl_publisher_t publisher_str;
std_msgs__msg__Int32 msg_int;
std_msgs__msg__String msg_str;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) delay(100);
  Serial.println("error");
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher_int, &msg_int, NULL));
    msg_int.data++;
  }
}

void publish_string(const char * text) {
  msg_str.data.data = (char *)text;
  msg_str.data.size = strlen(text);
  msg_str.data.capacity = msg_str.data.size + 1;
  rcl_publish(&publisher_str, &msg_str, NULL);
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();

  servo.attach(SERVO_PIN);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LEDPIN_R, OUTPUT);
  pinMode(LEDPIN_G, OUTPUT);
  pinMode(LEDPIN_B, OUTPUT);
  digitalWrite(LEDPIN_R, HIGH);
  digitalWrite(LEDPIN_G, HIGH);
  digitalWrite(LEDPIN_B, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher_int,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "reaction_time"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_str,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "reaction_text"));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg_int.data = 0;
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    digitalWrite(LEDPIN_R, LOW);
    digitalWrite(LEDPIN_G, LOW);
    digitalWrite(LEDPIN_B, LOW);

    Serial.println("Wait for it");
    publish_string("Wait for it");

    delay(random(2000, 5000));
    digitalWrite(LEDPIN_R, HIGH);
    digitalWrite(LEDPIN_G, HIGH);
    digitalWrite(LEDPIN_B, HIGH);


    unsigned long startTime = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {}
    unsigned long reactionTime = millis() - startTime;

    msg_int.data = reactionTime;
    rcl_publish(&publisher_int, &msg_int, NULL);

    int angle = map(reactionTime, 0, 1000, 180, 0);
    angle = constrain(angle, 0, 180);
    servo.write(angle);

    if (reactionTime <= 200) {
      Serial.println("Legendary");
      publish_string("Legendary");
    } 
    else if (reactionTime <= 299) {
      Serial.println("Average");
      publish_string("Average");
    } 
    else {
      Serial.println("Slow");
      publish_string("Slow");
    }

    delay(2000);
    servo.write(180);
  }

  delay(20);
}
