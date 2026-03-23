#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rover_interfaces/msg/rover_goal.hpp"
#include "rover_interfaces/srv/ocr_task.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include <random>

enum class MissionState { IDLE, SHUFFLING, GOING_TO_BOX1, SCANNING_BOX1, GOING_TO_BOX2, SCANNING_BOX2, FINAL_GOAL, COMPLETED };
struct Pose { double x, y, z, yaw; };

class MissionNode : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

  MissionNode() : Node("mission_control_node") {
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    ocr_client_ = this->create_client<rover_interfaces::srv::OcrTask>("/check_box_text");
    gz_client_ = this->create_client<SetEntityPose>("/world/mars_mission/set_pose");

    goal_sub_ = this->create_subscription<rover_interfaces::msg::RoverGoal>("/rover_goal", qos, std::bind(&MissionNode::goal_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/perseverance/odometry", qos, std::bind(&MissionNode::odom_callback, this, std::placeholders::_1));
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", qos, std::bind(&MissionNode::laser_callback, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/model/perseverance/camera", qos, std::bind(&MissionNode::camera_callback, this, std::placeholders::_1));    

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MissionNode::report, this));
    state_ = MissionState::IDLE;
    box_info = "UNKNOWN"; 
  }

private:
  void goal_callback(const rover_interfaces::msg::RoverGoal::SharedPtr msg) {
    if (state_ == MissionState::IDLE) {
      final_goal_x = msg->target_x; final_goal_y = msg->target_y;
      state_ = MissionState::SHUFFLING;
      shuffle_world();
    }
  }

  void shuffle_world() {
    Pose food_base = {-5.8618, 99.0651, 1.6335, 1.1627}; 
    Pose waste_base = {-100.99, 10.962, 1.9959, -0.0837};
    std::random_device rd; std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 1);

    if (dis(gen) == 1) {
        box_info = "BOXES CHANGE LOCATION";
        call_gz_teleport("food_box", waste_base);
        call_gz_teleport("waste_box", food_base);
    } else {
        box_info = "BOXES STAY";
        call_gz_teleport("food_box", food_base);
        call_gz_teleport("waste_box", waste_base);
    }
    state_ = MissionState::GOING_TO_BOX1;
    
    send_nav_goal(-8.6182, 80.1734, 0.6242, 0.7812); 
  }

  void send_nav_goal(float x, float y, float qz, float qw) {
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = x; goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.z = qz; goal_msg.pose.pose.orientation.w = qw;

    auto opt = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opt.result_callback = [this](const GoalHandleNav::WrappedResult & r) {
      if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (state_ == MissionState::GOING_TO_BOX1 || state_ == MissionState::GOING_TO_BOX2) {
          state_ = (state_ == MissionState::GOING_TO_BOX1) ? MissionState::SCANNING_BOX1 : MissionState::SCANNING_BOX2;
          call_ocr_service();
        } else if (state_ == MissionState::FINAL_GOAL) state_ = MissionState::COMPLETED;
      }
    };
    nav_client_->async_send_goal(goal_msg, opt);
  }

  void call_ocr_service() {
    auto req = std::make_shared<rover_interfaces::srv::OcrTask::Request>();
    req->start_scan = true;
    ocr_client_->async_send_request(req, [this](rclcpp::Client<rover_interfaces::srv::OcrTask>::SharedFuture future) {
      auto res = future.get();
      if (res->success) {
        RCLCPP_INFO(this->get_logger(), "OCR RESULT: %s", res->text.c_str());
        if (res->text.find("FOOD") != std::string::npos) {
          RCLCPP_WARN(this->get_logger(), "FOOD FOUND! Proceeding to final goal.");
          state_ = MissionState::FINAL_GOAL;
          send_nav_goal(final_goal_x, final_goal_y, 0.0, 1.0);
        } else {
          RCLCPP_ERROR(this->get_logger(), "WASTE DETECTED OR UNKNOWN. Searching further...");
          if (state_ == MissionState::SCANNING_BOX1) {
            state_ = MissionState::GOING_TO_BOX2;
            send_nav_goal(-79.9447, 11.9172, -0.9961, 0.0883); 
          }
        }
      }
    });
  }

  void report() {
    std::string s;
    switch(state_) {
        case MissionState::IDLE: s = "WAITING"; break;
        case MissionState::GOING_TO_BOX1: s = "NAV TO BOX 1"; break;
        case MissionState::SCANNING_BOX1: s = "OCR BOX 1"; break;
        case MissionState::GOING_TO_BOX2: s = "NAV TO BOX 2"; break;
        case MissionState::SCANNING_BOX2: s = "OCR BOX 2"; break;
        case MissionState::FINAL_GOAL: s = "NAV TO BASE"; break;
        case MissionState::COMPLETED: s = "MISSION COMPLETE"; break;
        default: s = "UNKNOWN";
    }
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "STATUS    : %s", s.c_str());
    RCLCPP_INFO(this->get_logger(), "SWAP      : %s", box_info.c_str());
    RCLCPP_INFO(this->get_logger(), "LASER MIN : %.2f m", absolute_min);
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
  }

  void call_gz_teleport(std::string name, Pose p) {
    auto req = std::make_shared<SetEntityPose::Request>();
    req->entity.name = name; req->entity.type = 2;
    req->pose.position.x = p.x; req->pose.position.y = p.y; req->pose.position.z = p.z;
    req->pose.orientation.z = std::sin(p.yaw / 2.0); req->pose.orientation.w = std::cos(p.yaw / 2.0);
    gz_client_->async_send_request(req);
  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { curr_x = msg->pose.pose.position.x; curr_y = msg->pose.pose.position.y; }
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
    float m = msg->range_max; for (auto r : msg->ranges) if (std::isfinite(r) && r < m && r > msg->range_min) m = r;
    absolute_min = m; 
  }
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) { try { cv::imshow("Rover Vision", cv_bridge::toCvCopy(msg, "bgr8")->image); cv::waitKey(1); } catch (...) {} }

  MissionState state_; std::string box_info;
  double final_goal_x, final_goal_y, curr_x, curr_y; float absolute_min = 0;
  rclcpp::Subscription<rover_interfaces::msg::RoverGoal>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Client<rover_interfaces::srv::OcrTask>::SharedPtr ocr_client_;
  rclcpp::Client<SetEntityPose>::SharedPtr gz_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<MissionNode>()); rclcpp::shutdown(); return 0; }