#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/uwbmeasurement.hpp"
#include "uav_interfaces/msg/groundtruth.hpp"

using std::placeholders::_1;

class DualSubscriber : public rclcpp::Node
{
  public:
    DualSubscriber()
    : Node("dual_subscriber")
    {
      subscription_1 = this->create_subscription<uav_interfaces::msg::Groundtruth>(
      "topic1", 10, std::bind(&DualSubscriber::topic_callback_gt, this, _1));
	  
	  subscription_2 = this->create_subscription<uav_interfaces::msg::Uwbmeasurement>(
      "topic2", 10, std::bind(&DualSubscriber::topic_callback_uwb, this, _1));
    }

  private:
    void topic_callback_gt(const uav_interfaces::msg::Groundtruth & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Frame: %d X: %f Y: %f Z:%f psi: %f", msg.frame_id, msg.x, msg.y, msg.z, msg.psi);
    }
	void topic_callback_uwb(const uav_interfaces::msg::Uwbmeasurement & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Frame: %d B:%f C:%f D:%f", msg.frame_id, msg.dist_b, msg.dist_c, msg.dist_d);
    }
    rclcpp::Subscription<uav_interfaces::msg::Groundtruth>::SharedPtr subscription_1;
	rclcpp::Subscription<uav_interfaces::msg::Uwbmeasurement>::SharedPtr subscription_2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dual_subscriber");
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
