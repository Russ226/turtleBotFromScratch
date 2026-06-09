#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

enum DIR{
  FORWARD_LEFT,
  FORWARD_RIGHT,
  BACK_LEFT,
  BACK_RIGHT,
  FORWARD,
  BACK,
  STOP
};


rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

int cur_dir = -1;

#define LED_PIN 38
#define ENR 12 
#define r_gpio_p 4
#define r_gpio_n 3
#define ENL 13
#define l_gpio_p 2
#define l_gpio_n 1

#define PWM_FREQ 1000
#define PWM_RES  8 
#define MAX_PWN 255
#define STEP_SIZE 10
#define STEP_DELAY 25

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}



void error_loop(){
  while(1){
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
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





void setDir(DIR d){
  switch(d){
    case FORWARD_LEFT:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, HIGH);
      digitalWrite(l_gpio_n, LOW);
      buildSpeed(ENL);
      break;
    case BACK_LEFT:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, HIGH);
      buildSpeed(ENL);
      break;
    case FORWARD_RIGHT:
      digitalWrite(r_gpio_p, HIGH);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, LOW);
      buildSpeed(ENR);
      break;
    case BACK_RIGHT:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, HIGH);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, LOW);
      buildSpeed(ENR);
      break;
    case FORWARD:
      digitalWrite(r_gpio_p, HIGH);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, HIGH);
      digitalWrite(l_gpio_n, LOW);
      setBuildSpeedBothSides();
      break;
    case BACK:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, HIGH);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, HIGH);
      setBuildSpeedBothSides();
      break;
    case STOP:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, LOW);
      setPwn(ENR, 0);
      setPwn(ENL, 0);
      break;
    default:
      digitalWrite(r_gpio_p, LOW);
      digitalWrite(r_gpio_n, LOW);
      digitalWrite(l_gpio_p, LOW);
      digitalWrite(l_gpio_n, LOW);
      setPwn(ENR, 0);
      setPwn(ENL, 0);
      break;

  }
}


void buildSpeed(int s){
  for(int i = 0; i < MAX_PWN; i += STEP_SIZE){
    setPwn(s, i);
    delay(STEP_DELAY);
  }
}

void setBuildSpeedBothSides(){
  for(int i = 0; i < MAX_PWN; i += STEP_SIZE){
    setPwn(ENR, i);
    setPwn(ENL, i);
    delay(STEP_DELAY);
  }
}
void setPwn(int s, int n){
  if(n > MAX_PWN) n = MAX_PWN;
  if(n < 0) n = 0;

  ledcWrite(s, n);
}


void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // add proper 
  // Serial.println(msg->data);
  switch(msg->data){
    case 1:
      setDir(FORWARD_LEFT);
      break;
    case 2:
      setDir(BACK_LEFT);
      break;
    case 3:
      setDir(FORWARD_RIGHT);
      break;
    case 4:
      setDir(BACK_RIGHT);
      break;
    case 5:
      setDir(FORWARD);
      break;
    case 6:
      setDir(BACK);
      break;
    case -1:
      setDir(STOP);
      break;
    default:
      setDir(STOP);
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
  ledcAttach(ENR, PWM_FREQ, PWM_RES);
  ledcAttach(ENL, PWM_FREQ, PWM_RES);
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
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  //Serial.println("testst");
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
