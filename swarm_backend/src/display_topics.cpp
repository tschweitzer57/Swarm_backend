#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/groundtruth.hpp"
#include "uav_interfaces/msg/viomeasurement.hpp"
#include "uav_interfaces/msg/uwbmeasurement.hpp"
#include "uav_interfaces/msg/vcmeasurement.hpp"
#include "uav_interfaces/msg/lcmeasurement.hpp"

using std::placeholders::_1;

class GroundtruthSubscriber : public rclcpp::Node
{
  public:
    GroundtruthSubscriber() : Node("gt_subscriber")
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

class VioSubscriber : public rclcpp::Node
{
  public:
    VioSubscriber() : Node("vio_subscriber")
    {
      // Create subscription for visual inertial odometry topic
	  subscription_ = this->create_subscription<uav_interfaces::msg::Viomeasurement>(
      "topic2", 10, std::bind(&VioSubscriber::topic_callback_vio, this, _1));
    }

  private:
    rclcpp::Subscription<uav_interfaces::msg::Viomeasurement>::SharedPtr subscription_;
	
	void topic_callback_vio(const uav_interfaces::msg::Viomeasurement & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Frame: %d data1: %f", msg.frame_id, msg.measure[1]);
    }   
};

class UwbSubscriber : public rclcpp::Node
{
  public:
    UwbSubscriber() : Node("uwb_subscriber")
    {
      // Create subscription for uwb topic
	  subscription_ = this->create_subscription<uav_interfaces::msg::Uwbmeasurement>(
      "topic3", 10, std::bind(&UwbSubscriber::topic_callback_uwb, this, _1));
    }

  private:
	rclcpp::Subscription<uav_interfaces::msg::Uwbmeasurement>::SharedPtr subscription_;
	
	void topic_callback_uwb(const uav_interfaces::msg::Uwbmeasurement & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Frame: %d B:%f C:%f D:%f", msg.frame_id, msg.dist_b, msg.dist_c, msg.dist_d);
    }
};

class VdSubscriber : public rclcpp::Node
{
  public:
    VdSubscriber() : Node("vd_subscriber")
    {
      // Create subscription for visual detection topic
	  subscription_ = this->create_subscription<uav_interfaces::msg::Vdmeasurement>(
      "topic4", 10, std::bind(&VdSubscriber::topic_callback_vd, this, _1));
    }

  private:
    rclcpp::Subscription<uav_interfaces::msg::Vdmeasurement>::SharedPtr subscription_;
	
	void topic_callback_vd(const uav_interfaces::msg::Vdmeasurement & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Frame: %d B detected:%d B measure:%f", msg.frame_id, msg.b_detected, msg.measure_b[1]);
    }   
};

class LcSubscriber : public rclcpp::Node
{
  public:
    LcSubscriber() : Node("lc_subscriber")
    {
      // Create subscription for loop closure topic
	  subscription_ = this->create_subscription<uav_interfaces::msg::Lcmeasurement>(
      "topic5", 10, std::bind(&LcSubscriber::topic_callback_lc, this, _1));
    }

  private:
    rclcpp::Subscription<uav_interfaces::msg::Lcmeasurement>::SharedPtr subscription_;
	
	void topic_callback_lc(const uav_interfaces::msg::Lcmeasurement & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Frame:%d detected:%d time:%d link:%s measure:%f", msg.frame_id, msg.detected_akf, msg.time_akf, msg.link_akf.c_str(), msg.measure_akf[1]);
    }   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<GroundtruthSubscriber> gt_node = std::make_shared<GroundtruthSubscriber>();
  std::shared_ptr<VioSubscriber> vio_node = std::make_shared<VioSubscriber>();
  std::shared_ptr<UwbSubscriber> uwb_node = std::make_shared<UwbSubscriber>();
  std::shared_ptr<VdSubscriber> vd_node = std::make_shared<VdSubscriber>();
  std::shared_ptr<LcSubscriber> lc_node = std::make_shared<LcSubscriber>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  
  executor.add_node(gt_node);
  executor.add_node(vio_node);
  executor.add_node(uwb_node);
  executor.add_node(vd_node);
  executor.add_node(lc_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
