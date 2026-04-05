#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>

static const char *TAG = "MICROROS";
static const int DOMAIN_ID = 23;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "FAIL");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
void app_main(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t publisher;
    std_msgs__msg__Int32 msg;

    //create init_options
   rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
   RCCHECK(rcl_init_options_init(&init_options, allocator));
   RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));
 
   // Create support with these options
   RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &    allocator));

    rclc_node_init_default(&node, "esp_publisher", "", &support);
    rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "counter");

    msg.data = 0;
    while (1) {
        msg.data++;
        rcl_publish(&publisher, &msg, NULL);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}