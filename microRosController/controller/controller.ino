#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float64_multi_array.h>

#include <Wire.h> 
#include <SparkFun_MMA8452Q.h>


MMA8452Q accel;
rcl_publisher_t vectorPublisher;
geometry_msgs__msg__Vector3 vectorMsg;
rclc_support_t vectorSupport;
rcl_allocator_t vectorAllocator;
rcl_node_t vectorNode;

rcl_publisher_t proximityPublisher;
std_msgs__msg__Float64MultiArray proximityMsg;
rclc_support_t proximitySupport;
rcl_allocator_t proximityAllocator;
rcl_node_t proximityNode;


#if defined(LED_BUILTIN)
  #define LED_PIN LED_BUILTIN
#else
  #define LED_PIN 13
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&vectorPublisher, &vectorMsg, NULL));
    RCSOFTCHECK(rcl_publish(&proximityPublisher, &proximityMsg, NULL));
  }
}

void createImuNode(){
  vectorAllocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&vectorSupport, 0, NULL, &vectorAllocator));

  // create node
  RCCHECK(rclc_node_init_default(&vectorNode, "micro_ros_vector3", "", &vectorSupport));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &vectorPublisher,
    &vectorNode,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "data_imu"));
}

void createProximityNode(){
  proximityAllocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&proximitySupport, 0, NULL, &proximityAllocator));

  // create node
  RCCHECK(rclc_node_init_default(&proximityNode, "micro_ros_proximity", "", &proximitySupport));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &proximityPublisher,
    &proximityNode,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "data_proximity"));
}

void setup() {
  Serial.begin(115200); 
  set_microros_wifi_transports("Randy Lahey", "Agostini", "192.168.1.110", 8888);
  Wire.begin();
  accel.begin();
  //set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  createImuNode();
}

void loop() {
    if (accel.available()) { 
      accel.read(); 
      vectorMsg.x = accel.cx;
      vectorMsg.y = accel.cy;
      vectorMsg.z = accel.cz;
      
    } 
    
    RCSOFTCHECK(rcl_publish(&vectorPublisher, &vectorMsg, NULL));
    RCSOFTCHECK(rcl_publish(&proximityPublisher, &proximityMsg, NULL));
    
}
