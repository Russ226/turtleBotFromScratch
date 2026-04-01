 // all move 50ms
 // -1 == stop
 // 1  == move left forward motor
 // 2 == move left back motor
 // 3 == move right foward motor
 // 4 == move right backward motor
 // 5 == move both forward
 // 6 == move both backward


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
class AvoidanceCalc : public rclcpp::Node {
public:
  AvoidanceCalc(): Node("avoidanceCalc"){
    // auto qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&AvoidanceCalc::scanCallback, this, std::placeholders::_1));
    dirPb_ = create_publisher<std_msgs::msg::Int32>("/dir", 10);
    timer_ = create_wall_timer(50ms, std::bind(&AvoidanceCalc::dirCallback, this));

  }


private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dirPb_;
  rclcpp::TimerBase::SharedPtr timer_;
  int curDir_ = 5;
  float minDistance_ =0.50;

  double minValidRange(const sensor_msgs::msg::LaserScan::SharedPtr msg, int start_idx, int end_idx) {
    double min_range = std::numeric_limits<double>::infinity();

    start_idx = std::max(0, start_idx);
    end_idx = std::min(static_cast<int>(msg->ranges.size()) - 1, end_idx);

    for (int i = start_idx; i <= end_idx; ++i) {
      const float r = msg->ranges[i];
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        min_range = std::min(min_range, static_cast<double>(r));
      }
    }
    return min_range;
    }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    const int n = static_cast<int>(msg->ranges.size());
    if (n == 0) {
      return;
    }

    int center = n/2;
    int window = n/12;

    double front_min = minValidRange(msg, center - window, center + window);
    double left_min  = minValidRange(msg, center + window, center + 3 * window);
    double right_min = minValidRange(msg, center - 3 * window, center - window);
    double back_min = minValidRange(msg, center + window, center - window);

    if (front_min > minDistance_) {
        curDir_ = 5;
    }
    else if(left_min > minDistance_){
        curDir_ = 3;
    }
    else if(right_min > minDistance_){
        curDir_ = 1;
    }
    else if(back_min > minDistance_){
        curDir_ = 6;
    }else {
        curDir_ = -1;
    }
  }

  void dirCallback(){
    auto mes = std_msgs::msg::Int32();
    mes.data = curDir_;
    dirPb_->publish(mes);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidanceCalc>());
  rclcpp::shutdown();
  return 0;
}