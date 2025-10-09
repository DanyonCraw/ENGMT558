#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <ESP32Servo.h>


#define BUTTON_PIN 25
#define LEDPIN_R 14
#define LEDPIN_G 27
#define LEDPIN_B 26
#define SERVO_PIN  33

Servo servo;
rcl_publisher_t button_pub;
rcl_subscription_t led_sub;
rcl_subscription_t servo_sub;
rclc_executor_t executor;
rcl_node_t node;
rcl_timer_t timer;

std_msgs__msg__Bool button_msg;
std_msgs__msg__String led_msg;
std_msgs__msg__Float32 servo_msg;

// Timer callback publishes button state
// void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//     bool pressed = !digitalRead(BUTTON_PIN); // Active-low
//     button_msg.data = pressed;
//     rcl_publish(&button_pub, &button_msg, NULL);
//     Serial.println("sending");
// }

// LED subscriber callback
void led_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  if (strcmp(msg->data.data, "on") == 0 || strcmp(msg->data.data, "red") == 0){
    digitalWrite(LEDPIN_R, LOW);
    digitalWrite(LEDPIN_G, LOW);
    digitalWrite(LEDPIN_B, LOW);
  }
  else{
    digitalWrite(LEDPIN_R, HIGH);
    digitalWrite(LEDPIN_G, HIGH);
    digitalWrite(LEDPIN_B, HIGH);
  }
}

// Servo subscriber callback
void servo_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float angle = constrain(msg->data, 0, 180);
  servo.write(angle);
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LEDPIN_R, OUTPUT);
  pinMode(LEDPIN_G, OUTPUT);
  pinMode(LEDPIN_B, OUTPUT);
  digitalWrite(LEDPIN_R, HIGH);
  digitalWrite(LEDPIN_G, HIGH);
  digitalWrite(LEDPIN_B, HIGH);

  servo.attach(SERVO_PIN);

  Serial.begin(115200);
  // Transport over serial
  set_microros_transports();

  static rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  // Node
  rclc_node_init_default(&node, "esp32_reaction_node", "", &support);

  // Publisher
  rclc_publisher_init_default(
    &button_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "button_state");

  // Subscribers
  rclc_subscription_init_default(
    &led_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "led_command");

  rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "servo_angle");

  // Timer (publishes every 100 ms)
  //const unsigned int timer_timeout = 100;
  //rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
  bool pressed = !digitalRead(BUTTON_PIN); // Active-low
  button_msg.data = pressed;
  rcl_publish(&button_pub, &button_msg, NULL);
  Serial.println("sending");
  delay(10);
}
