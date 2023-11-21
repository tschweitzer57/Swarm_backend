#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/uwbmeasurement.hpp"
#include "uav_interfaces/msg/groundtruth.hpp"

using std::placeholders::_1;

class GroundtruthSubscriber : public rclcpp::Node
{
  public:
    GroundtruthSubscriber()
    : Node("groundtruth_subscriber")
    {
      // Create subscription for groundtruth topic
	  subscription_ = this->create_subscription<uav_interfaces::msg::Groundtruth>(
      "topic1", 10, std::bind(&GroundtruthSubscriber::topic_callback_gt, this, _1));
    }

  private:
    rclcpp::Subscription<uav_interfaces::msg::Groundtruth>::SharedPtr subscription_;
	
	void topic_callback_gt(const uav_interfaces::msg::Groundtruth & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Frame: %d X: %f Y: %f Z:%f psi: %f", msg.frame_id, msg.x, msg.y, msg.z, msg.psi);
    }   
};

class UwbSubscriber : public rclcpp::Node
{
  public:
    UwbSubscriber()
    : Node("uwb_subscriber")
    {
      // Create subscription for uwb topic
	  subscription_ = this->create_subscription<uav_interfaces::msg::Uwbmeasurement>(
      "topic2", 10, std::bind(&UwbSubscriber::topic_callback_uwb, this, _1));
    }

  private:
	rclcpp::Subscription<uav_interfaces::msg::Uwbmeasurement>::SharedPtr subscription_;
	
	void topic_callback_uwb(const uav_interfaces::msg::Uwbmeasurement & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Frame: %d B:%f C:%f D:%f", msg.frame_id, msg.dist_b, msg.dist_c, msg.dist_d);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<GroundtruthSubscriber> gt_node = std::make_shared<GroundtruthSubscriber>();
  std::shared_ptr<UwbSubscriber> uwb_node = std::make_shared<UwbSubscriber>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  
  executor.add_node(gt_node);
  executor.add_node(uwb_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
