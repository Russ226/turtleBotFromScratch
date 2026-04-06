#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13
#define r_gpio_p 2
#define r_gpio_n 3

#define l_gpio_p 0
#define l_gpio_n 1


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}



void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// all move 50ms
// -1 == stop
// 1  == move left forward motor
// 2 == move left back motor
// 3 == move right foward motor
// 4 == move right backward motor
// 5 == move both forward
// 6 == move both backward
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // add proper 
  Serial.println(msg->data);
  switch(msg->data){
    case 1:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, HIGH);
      digitalWrite(l_gpio_n, LOW);
      break;
    case 2:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, HIGH);
      break;
    case 3:
      digitalWrite(r_gpio_p, HIGH);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, LOW);
      break;
    case 4:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, HIGH);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, LOW);
      break;
    case 5:
      digitalWrite(r_gpio_p, HIGH);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, HIGH);
      digitalWrite(l_gpio_n, LOW);
      break;
    case 6:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, HIGH);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, HIGH);
      break;
    case -1:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, LOW);
      break;
    default:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, LOW);
      Serial.println("invalid move command");
      break;
  }

}

void setup() {
  set_microros_transports(); 
  Serial.begin(115200); 
  delay(250);

  // set_microros_wifi_transports("Randy Lahey", "Agostini", "192.168.1.110", 8999);
  // WiFi.begin("Randy Lahey", "Agostini");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println();
  // Serial.print("WiFi connected, IP: ");
  // Serial.println(WiFi.localIP());
  // Serial.println("Connecting to WiFi...");

  pinMode(LED_PIN, OUTPUT);
  pinMode(r_gpio_p, OUTPUT);
  pinMode(r_gpio_n, OUTPUT);
  pinMode(l_gpio_p, OUTPUT);
  pinMode(l_gpio_n, OUTPUT);
  digitalWrite(r_gpio_p, LOW);
  digitalWrite(r_gpio_n, LOW);
  digitalWrite(l_gpio_p, LOW);
  digitalWrite(l_gpio_n, LOW); 
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 23));

  // Create support with these options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "dir_recv", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "dir"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ALWAYS));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}