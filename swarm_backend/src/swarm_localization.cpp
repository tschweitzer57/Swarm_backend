#include <memory>
#include <thread>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <string>

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
    int frame_id;
	Vector4d pose;

    GroundtruthData(int id, Vector4d vec)
        : frame_id(id), pose(vec) {}
};

struct VioData {
	int frame_id;
	Matrix4d pose;
	
	VioData(int id, Matrix4d mat)
		: frame_id(id), pose(mat) {}
};

struct UwbData {
	int frame_id;
	double dist_B, dist_C, dist_D;
	
	UwbData(int id, double d_b, double d_c, double d_d)
		: frame_id(id), dist_B(d_b), dist_C(d_c), dist_D(d_d) {}
};
struct VdData {
	int frame_id;
	bool detection_B, detection_C, detection_D;
	Matrix4d measure_B, measure_C, measure_D;
	
	VdData(int id, bool det_b, bool det_c, bool det_d, Matrix4d m_b, Matrix4d m_c, Matrix4d m_d)
		: frame_id(id), detection_B(det_b), detection_C(det_c), detection_D(det_d), 
		measure_B(m_b), measure_C(m_c), measure_D(m_d) {}
};
struct LcData {
	int frame_id;
	bool detection_akf, detection_bkf, detection_ckf, detection_dkf;
	int time_akf, time_bkf, time_ckf, time_dkf;
	std::string link_akf, link_bkf, link_ckf, link_dkf;
	Matrix4d measure_akf, measure_bkf, measure_ckf, measure_dkf;
 	
	LcData(int id, bool d_akf, bool d_bkf, bool d_ckf, bool d_dkf,
	int t_akf, int t_bkf, int t_ckf, int t_dkf,
	std::string l_akf, std::string l_bkf, std::string l_ckf, std::string l_dkf,
	Matrix4d m_akf, Matrix4d m_bkf, Matrix4d m_ckf, Matrix4d m_dkf)
		: frame_id(id), detection_akf(d_akf), detection_bkf(d_bkf), detection_ckf(d_ckf), detection_dkf(d_dkf),
		time_akf(t_akf), time_bkf(t_bkf), time_ckf(t_ckf), time_dkf(t_dkf),
		link_akf(l_akf), link_bkf(l_bkf), link_ckf(l_ckf), link_dkf(l_dkf),
		measure_akf(m_akf), measure_bkf(m_bkf), measure_ckf(m_ckf), measure_dkf(m_dkf) {}
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
	std::vector<UwbData> uwb_db_;
	std::vector<VdData> vd_db_;
	std::vector<LcData> lc_db_;
	
	// Ceres variables
	ceres::Problem problem_;
	ceres::CostFunction* cost_function_ = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
	ceres::Solver::Options options_;
	ceres::Solver::Summary summary_;
	
	void topic_callback_gt(const uav_interfaces::msg::Groundtruth & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame: %d X: %f Y: %f Z:%f psi: %f", msg.frame_id, msg.x, msg.y, msg.z, msg.psi);
		gt_received_ = true;
		
		Vector4d vec(msg.x, msg.y, msg.z, msg.psi);
		gt_db_.emplace_back(msg.frame_id, vec);
		
		synch_topic();
    }
	
	void topic_callback_vio(const uav_interfaces::msg::Viomeasurement & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame: %d data1: %f", msg.frame_id, msg.measure[1]);
		vio_received_ = true;
		
		Matrix4d vio_measure;
		vio_measure << msg.measure[0], msg.measure[1], msg.measure[2], msg.measure[3],
					   msg.measure[4], msg.measure[5], msg.measure[6], msg.measure[7],
					   msg.measure[8], msg.measure[9], msg.measure[10], msg.measure[11],
					   msg.measure[12], msg.measure[13], msg.measure[14], msg.measure[15];
		
		vio_db_.emplace_back(msg.frame_id, vio_measure);
		
		synch_topic();
    }
	
	void topic_callback_uwb(const uav_interfaces::msg::Uwbmeasurement & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame: %d B:%f C:%f D:%f", msg.frame_id, msg.dist_b, msg.dist_c, msg.dist_d);
		uwb_received_ = true;
		
		uwb_db_.emplace_back(msg.frame_id, msg.dist_b, msg.dist_c, msg.dist_d)
		
		synch_topic();
    }
	
	void topic_callback_vd(const uav_interfaces::msg::Vdmeasurement & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame: %d B detected:%d B measure:%f", msg.frame_id, msg.b_detected, msg.measure_b[1]);
		vd_received_ = true;
		
		Matrix4d vd_measure_B, vd_measure_C, vd_measure_D;
		vd_measure_B << msg.measure_b[0], 	msg.measure_b[1], 	msg.measure_b[2], 	msg.measure_b[3],
					    msg.measure_b[4], 	msg.measure_b[5], 	msg.measure_b[6], 	msg.measure_b[7],
					    msg.measure_b[8], 	msg.measure_b[9], 	msg.measure_b[10], 	msg.measure_b[11],
					    msg.measure_b[12], 	msg.measure_b[13], 	msg.measure_b[14], 	msg.measure_b[15];
						
		vd_measure_C << msg.measure_c[0], 	msg.measure_c[1], 	msg.measure_c[2], 	msg.measure_c[3],
					    msg.measure_c[4], 	msg.measure_c[5], 	msg.measure_c[6], 	msg.measure_c[7],
					    msg.measure_c[8], 	msg.measure_c[9], 	msg.measure_c[10], 	msg.measure_c[11],
					    msg.measure_c[12], 	msg.measure_c[13], 	msg.measure_c[14], 	msg.measure_c[15];
						
		vd_measure_D << msg.measure_d[0], 	msg.measure_d[1], 	msg.measure_d[2], 	msg.measure_d[3],
					    msg.measure_d[4], 	msg.measure_d[5], 	msg.measure_d[6], 	msg.measure_d[7],
					    msg.measure_d[8], 	msg.measure_d[9], 	msg.measure_d[10], 	msg.measure_d[11],
					    msg.measure_d[12], 	msg.measure_d[13], 	msg.measure_d[14], 	msg.measure_d[15];
		
		vd_db_.emplace_back(msg.frame_id, msg.b_detected, msg.c_detected, msg.d_detected, vd_measure_B, vd_measure_C, vd_measure_D)
		
		synch_topic();
    }
	
	void topic_callback_lc(const uav_interfaces::msg::Lcmeasurement & msg)
    {
		//RCLCPP_INFO(this->get_logger(), "Frame:%d detected:%d time:%d link:%s measure:%f", msg.frame_id, msg.detected_akf, msg.time_akf, msg.link_akf.c_str(), msg.measure_akf[1]);
		lc_received_ = true;
		
		Matrix4d lc_measure_akf, lc_measure_bkf, lc_measure_ckf, lc_measure_dkf;
		lc_measure_akf << 	msg.measure_akf[0], 	msg.measure_akf[1], 	msg.measure_akf[2], 	msg.measure_akf[3],
							msg.measure_akf[4], 	msg.measure_akf[5], 	msg.measure_akf[6], 	msg.measure_akf[7],
							msg.measure_akf[8], 	msg.measure_akf[9], 	msg.measure_akf[10], 	msg.measure_akf[11],
							msg.measure_akf[12], 	msg.measure_akf[13], 	msg.measure_akf[14], 	msg.measure_akf[15];
		
		lc_measure_bkf << 	msg.measure_bkf[0], 	msg.measure_bkf[1], 	msg.measure_bkf[2], 	msg.measure_bkf[3],
							msg.measure_bkf[4], 	msg.measure_bkf[5], 	msg.measure_bkf[6], 	msg.measure_bkf[7],
							msg.measure_bkf[8], 	msg.measure_bkf[9], 	msg.measure_bkf[10], 	msg.measure_bkf[11],
							msg.measure_bkf[12], 	msg.measure_bkf[13], 	msg.measure_bkf[14], 	msg.measure_bkf[15];
						
		lc_measure_ckf << 	msg.measure_ckf[0], 	msg.measure_ckf[1], 	msg.measure_ckf[2], 	msg.measure_ckf[3],
							msg.measure_ckf[4], 	msg.measure_ckf[5], 	msg.measure_ckf[6], 	msg.measure_ckf[7],
							msg.measure_ckf[8], 	msg.measure_ckf[9], 	msg.measure_ckf[10], 	msg.measure_ckf[11],
							msg.measure_ckf[12], 	msg.measure_ckf[13], 	msg.measure_ckf[14], 	msg.measure_ckf[15];
						
		lc_measure_dkf << 	msg.measure_dkf[0], 	msg.measure_dkf[1], 	msg.measure_dkf[2], 	msg.measure_dkf[3],
							msg.measure_dkf[4], 	msg.measure_dkf[5], 	msg.measure_dkf[6], 	msg.measure_dkf[7],
							msg.measure_dkf[8], 	msg.measure_dkf[9], 	msg.measure_dkf[10], 	msg.measure_dkf[11],
							msg.measure_dkf[12], 	msg.measure_dkf[13], 	msg.measure_dkf[14], 	msg.measure_dkf[15];
		
		lc_db_.emplace_back(msg.frame_id, msg.detected_akf, msg.detected_bkf, msg.detection_ckf, msg.detection_dkf,
							msg.time_akf, msg.time_bkf, msg.time_ckf, msg.time_dkf,
							msg.link_akf, msg.link_bkf, msg.link_ckf, msg.link_dkf,
							lc_measure_akf, lc_measure_bkf, lc_measure_ckf, lc_measure_dkf)
		
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
			RCLCPP_INFO(this->get_logger(), "GT data vector size is : %ld", gt_db_.size());
			RCLCPP_INFO(this->get_logger(), "VIO data vector size is : %ld", vio_db_.size());
			RCLCPP_INFO(this->get_logger(), "UWB data vector size is : %ld", uwb_db_.size());
			RCLCPP_INFO(this->get_logger(), "VD data vector size is : %ld", vd_db_.size());
			RCLCPP_INFO(this->get_logger(), "LC data vector size is : %ld", lc_db_.size());
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
