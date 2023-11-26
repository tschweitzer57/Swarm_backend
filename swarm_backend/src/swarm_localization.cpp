#include <memory>
#include <thread>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/groundtruth.hpp"
#include "uav_interfaces/msg/viomeasurement.hpp"
#include "uav_interfaces/msg/uwbmeasurement.hpp"
#include "uav_interfaces/msg/vdmeasurement.hpp"
#include "uav_interfaces/msg/lcmeasurement.hpp"

#include "ceres/ceres.h"
#include "glog/logging.h"

using std::placeholders::_1;
using namespace Eigen;

    // Sleep for 1 second (simulating a periodic update)
    //std::this_thread::sleep_for(std::chrono::seconds(1));
	//std::chrono::system_clock::to_time_t(point.timestamp)
    
struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

struct GroundtruthData {
    Vector4d pose;
    int frame_id;

    GroundtruthData(int id, Vector4d vec)
        : frame_id(id), pose(vec) {}
};

struct VioData {
	Matrix4d pose;
	int frame_id;
	
	VioData(int id, Matrix4d mat)
		: frame_id(id), pose(mat) {}
};

class UavSubscriber : public rclcpp::Node
{
  public:
    UavSubscriber() : Node("uav_subscriber")
    {
      // Create subscription for groundtruth topic
	  gt_sub_ = this->create_subscription<uav_interfaces::msg::Groundtruth>(
      "topic1", 10, std::bind(&UavSubscriber::topic_callback_gt, this, _1));
	  
	  // Create subscription for visual inertial odometry topic
	  vio_sub_ = this->create_subscription<uav_interfaces::msg::Viomeasurement>(
      "topic2", 10, std::bind(&UavSubscriber::topic_callback_vio, this, _1));
	  
	  // Create subscription for uwb topic
	  uwb_sub_ = this->create_subscription<uav_interfaces::msg::Uwbmeasurement>(
      "topic3", 10, std::bind(&UavSubscriber::topic_callback_uwb, this, _1));
	  
	  // Create subscription for visual detection topic
	  vd_sub_ = this->create_subscription<uav_interfaces::msg::Vdmeasurement>(
      "topic4", 10, std::bind(&UavSubscriber::topic_callback_vd, this, _1));
	  
	  // Create subscription for loop closure topic
	  lc_sub_ = this->create_subscription<uav_interfaces::msg::Lcmeasurement>(
      "topic5", 10, std::bind(&UavSubscriber::topic_callback_lc, this, _1));
    }

  private:
    rclcpp::Subscription<uav_interfaces::msg::Groundtruth>::SharedPtr gt_sub_;
	rclcpp::Subscription<uav_interfaces::msg::Viomeasurement>::SharedPtr vio_sub_;
	rclcpp::Subscription<uav_interfaces::msg::Uwbmeasurement>::SharedPtr uwb_sub_;
	rclcpp::Subscription<uav_interfaces::msg::Vdmeasurement>::SharedPtr vd_sub_;
	rclcpp::Subscription<uav_interfaces::msg::Lcmeasurement>::SharedPtr lc_sub_;
	
	bool gt_received_ {false};
	bool vio_received_ {false};
	bool uwb_received_ {false};
	bool vd_received_ {false};
	bool lc_received_ {false};
	
	std::vector<GroundtruthData> gt_db_;
	std::vector<VioData> vio_db_;
	
	// Ceres variables
	ceres::Problem problem_;
	ceres::CostFunction* cost_function_ = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
	ceres::Solver::Options options_;
	ceres::Solver::Summary summary_;
	
	void topic_callback_gt(const uav_interfaces::msg::Groundtruth & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame: %d X: %f Y: %f Z:%f psi: %f", msg.frame_id, msg.x, msg.y, msg.z, msg.psi);
		gt_received_ = true;
		
		int id = msg.frame_id;
		Vector4d vec(msg.x, msg.y, msg.z, msg.psi);
		gt_db_.emplace_back(id, vec);
		
		synch_topic();
    }
	
	void topic_callback_vio(const uav_interfaces::msg::Viomeasurement & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame: %d data1: %f", msg.frame_id, msg.measure[1]);
		vio_received_ = true;
		
		int id = msg.frame_id;
		Matrix4d vio_measure 
		{msg.measure[0], msg.measure[1], msg.measure[2], msg.measure[3],
		 msg.measure[4], msg.measure[5], msg.measure[6], msg.measure[7],
		 msg.measure[8], msg.measure[9], msg.measure[10], msg.measure[11],
		 msg.measure[12], msg.measure[13], msg.measure[14], msg.measure[15]};
		
		vio_db_.emplace_back(id, vio_measure)
		
		synch_topic();
    }
	
	void topic_callback_uwb(const uav_interfaces::msg::Uwbmeasurement & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame: %d B:%f C:%f D:%f", msg.frame_id, msg.dist_b, msg.dist_c, msg.dist_d);
		uwb_received_ = true;
		synch_topic();
    }
	
	void topic_callback_vd(const uav_interfaces::msg::Vdmeasurement & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame: %d B detected:%d B measure:%f", msg.frame_id, msg.b_detected, msg.measure_b[1]);
		vd_received_ = true;
		synch_topic();
    }
	
	void topic_callback_lc(const uav_interfaces::msg::Lcmeasurement & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame:%d detected:%d time:%d link:%s measure:%f", msg.frame_id, msg.detected_akf, msg.time_akf, msg.link_akf.c_str(), msg.measure_akf[1]);
		lc_received_ = true;
		synch_topic();
    }
	
	void synch_topic()
	{
		// Check if sliding window is complete
		if(gt_received_ && vio_received_ && uwb_received_ && vd_received_ && lc_received_)
		{
			// slidingwindow()
			// optimize()
			RCLCPP_INFO(this->get_logger(), "----------ALL MESSAGES RECEIVED-------------");
			gt_received_ = false;
			vio_received_ = false;
			uwb_received_ = false;
			vd_received_ = false;
			lc_received_ = false;
			
			double x = 0.5;
			const double initial_x = x;
	  
			// Set up the only cost function (also known as residual). This uses
			// auto-differentiation to obtain the derivative (jacobian).
			
			problem_.AddResidualBlock(cost_function_, nullptr, &x);
			// Run the solver!
	  
			options_.minimizer_progress_to_stdout = true;
			
			ceres::Solve(options_, &problem_, &summary_);
			//std::cout << summary_.BriefReport() << "\n";
			//std::cout << "x : " << initial_x << " -> " << x << "\n";
			RCLCPP_INFO(this->get_logger(), "x : %f -> %f", initial_x, x);
			RCLCPP_INFO(this->get_logger(), "GT data vector size is : %d", gt_db_.size());
			RCLCPP_INFO(this->get_logger(), "VIO data vector size is : %d", vio_db_.size());
		}
	}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  
  std::shared_ptr<UavSubscriber> uav_node = std::make_shared<UavSubscriber>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  
  executor.add_node(uav_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
