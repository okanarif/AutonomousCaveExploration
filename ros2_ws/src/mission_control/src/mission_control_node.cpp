// =============================================================================
// Mission Control Node — Finite State Machine for Cave Navigation
// =============================================================================
//
// Orchestrates the autonomous cave navigation mission through a sequence of
// states: IDLE → TAKEOFF → NAVIGATE_TO_CAVE → EXPLORE_CAVE → FINISHED.
//
// In each state, the node either publishes desired_state commands directly
// (IDLE, TAKEOFF, FINISHED) or delegates control to other nodes via ROS 2
// service calls (NAVIGATE_TO_CAVE → sampling_planner, EXPLORE_CAVE →
// frontier_detector).
//
// All parameters (altitudes, tolerances, topic/service names, waypoints)
// are loaded from a YAML config file at startup.
// =============================================================================

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <future>

#include "mission_interfaces/srv/execute_trajectory.hpp"

#include <Eigen/Dense>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// -----------------------------------------------------------------------------
// Mission state definitions
// -----------------------------------------------------------------------------
enum class MissionState {
  IDLE,               // Wait on ground before starting
  TAKEOFF,            // Climb to target altitude
  NAVIGATE_TO_CAVE,   // Follow waypoints to cave entrance via trajectory planner
  EXPLORE_CAVE,       // Hand off to frontier-based exploration
  FINISHED            // Hold position after mission completion
};

std::string stateToString(MissionState state) {
  switch(state) {
    case MissionState::IDLE:              return "IDLE";
    case MissionState::TAKEOFF:           return "TAKEOFF";
    case MissionState::NAVIGATE_TO_CAVE:  return "NAVIGATE_TO_CAVE";
    case MissionState::EXPLORE_CAVE:      return "EXPLORE_CAVE";
    case MissionState::FINISHED:          return "FINISHED";
    default:                              return "UNKNOWN";
  }
}

// =============================================================================
// MissionControlNode
// =============================================================================
class MissionControlNode : public rclcpp::Node {
private:
  // ---- State machine --------------------------------------------------------
  MissionState current_state_;

  // ---- ROS communication ----------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trajectory_complete_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Client<mission_interfaces::srv::ExecuteTrajectory>::SharedPtr start_navigation_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_exploration_client_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  // ---- Current UAV state (from odometry) ------------------------------------
  Eigen::Vector3d current_position_;
  Eigen::Vector3d current_velocity_;
  double current_yaw_;
  double desired_yaw_;
  bool received_current_state_;

  // ---- Mission parameters (loaded from YAML) --------------------------------
  double takeoff_altitude_;
  double position_tolerance_;
  double velocity_tolerance_;
  double state_machine_hz_;
  double idle_duration_;

  // ---- Cave entrance waypoints ----------------------------------------------
  std::vector<double> cave_waypoints_x_;
  std::vector<double> cave_waypoints_y_;
  std::vector<double> cave_waypoints_z_;

  // ---- Navigation flags -----------------------------------------------------
  bool navigation_triggered_;
  bool trajectory_completed_;
  bool exploration_triggered_;

  // ---- Timing ---------------------------------------------------------------
  rclcpp::Time state_start_time_;
  rclcpp::Time idle_start_time_;

public:
  MissionControlNode()
  : Node("mission_control_node"),
    current_state_(MissionState::IDLE),
    current_position_(0, 0, 0),
    current_velocity_(0, 0, 0),
    current_yaw_(0.0),
    desired_yaw_(0.0),
    received_current_state_(false),
    navigation_triggered_(false),
    trajectory_completed_(false),
    exploration_triggered_(false)
  {
    // ------------------------------------------------------------------
    // Declare parameters with default values
    // ------------------------------------------------------------------
    // Mission behaviour
    this->declare_parameter<double>("takeoff_altitude", 9.0);
    this->declare_parameter<double>("position_tolerance", 0.3);
    this->declare_parameter<double>("velocity_tolerance", 0.1);
    this->declare_parameter<double>("state_machine_hz", 10.0);
    this->declare_parameter<double>("idle_duration", 3.0);

    // Cave entrance waypoints (default: empty — must be provided in config)
    this->declare_parameter<std::vector<double>>("cave_entrance_waypoints.x", std::vector<double>());
    this->declare_parameter<std::vector<double>>("cave_entrance_waypoints.y", std::vector<double>());
    this->declare_parameter<std::vector<double>>("cave_entrance_waypoints.z", std::vector<double>());

    // Topic names
    this->declare_parameter<std::string>("current_state_topic", "current_state_est");
    this->declare_parameter<std::string>("desired_state_topic", "desired_state");
    this->declare_parameter<std::string>("trajectory_complete_topic", "trajectory_complete");
    this->declare_parameter<std::string>("state_mode_topic", "stm_mode");

    // Service names
    this->declare_parameter<std::string>("start_navigation_service", "start_navigation");
    this->declare_parameter<std::string>("start_exploration_service", "start_exploration");

    // Queue sizes
    this->declare_parameter<int>("subscriber_queue_size", 10);
    this->declare_parameter<int>("publisher_queue_size", 10);

    // ------------------------------------------------------------------
    // Load and validate parameters
    // ------------------------------------------------------------------
    if (!loadParameters()) {
      RCLCPP_FATAL(this->get_logger(),
        "Failed to load required parameters. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    // ------------------------------------------------------------------
    // Retrieve topic / service names and queue sizes
    // ------------------------------------------------------------------
    std::string current_state_topic     = this->get_parameter("current_state_topic").as_string();
    std::string desired_state_topic     = this->get_parameter("desired_state_topic").as_string();
    std::string trajectory_complete_topic = this->get_parameter("trajectory_complete_topic").as_string();
    std::string state_mode_topic        = this->get_parameter("state_mode_topic").as_string();
    std::string start_navigation_service  = this->get_parameter("start_navigation_service").as_string();
    std::string start_exploration_service = this->get_parameter("start_exploration_service").as_string();
    int sub_queue = this->get_parameter("subscriber_queue_size").as_int();
    int pub_queue = this->get_parameter("publisher_queue_size").as_int();

    // ------------------------------------------------------------------
    // Set up subscribers
    // ------------------------------------------------------------------
    current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      current_state_topic, sub_queue,
      std::bind(&MissionControlNode::onCurrentState, this, std::placeholders::_1));

    trajectory_complete_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      trajectory_complete_topic, sub_queue,
      std::bind(&MissionControlNode::onTrajectoryComplete, this, std::placeholders::_1));

    // ------------------------------------------------------------------
    // Set up publishers
    // ------------------------------------------------------------------
    desired_state_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      desired_state_topic, pub_queue);
    state_pub_ = this->create_publisher<std_msgs::msg::String>(state_mode_topic, 10);

    // ------------------------------------------------------------------
    // Set up service clients
    // ------------------------------------------------------------------
    start_navigation_client_ =
      this->create_client<mission_interfaces::srv::ExecuteTrajectory>(start_navigation_service);
    start_exploration_client_ =
      this->create_client<std_srvs::srv::Trigger>(start_exploration_service);

    // ------------------------------------------------------------------
    // Start the state machine timer
    // ------------------------------------------------------------------
    state_machine_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / state_machine_hz_),
      std::bind(&MissionControlNode::stateMachineLoop, this));

    state_start_time_ = this->now();
    idle_start_time_  = this->now();

    RCLCPP_INFO(this->get_logger(),
      "Mission Control initialized (%.0f Hz, takeoff=%.1f m, %zu waypoints)",
      state_machine_hz_, takeoff_altitude_, cave_waypoints_x_.size());
  }

private:
  // ===========================================================================
  // Parameter loading and validation
  // ===========================================================================
  bool loadParameters() {
    try {
      takeoff_altitude_   = this->get_parameter("takeoff_altitude").as_double();
      position_tolerance_ = this->get_parameter("position_tolerance").as_double();
      velocity_tolerance_ = this->get_parameter("velocity_tolerance").as_double();
      state_machine_hz_   = this->get_parameter("state_machine_hz").as_double();
      idle_duration_      = this->get_parameter("idle_duration").as_double();

      cave_waypoints_x_ = this->get_parameter("cave_entrance_waypoints.x").as_double_array();
      cave_waypoints_y_ = this->get_parameter("cave_entrance_waypoints.y").as_double_array();
      cave_waypoints_z_ = this->get_parameter("cave_entrance_waypoints.z").as_double_array();

      // Validate
      if (takeoff_altitude_ <= 0) {
        RCLCPP_ERROR(this->get_logger(), "takeoff_altitude must be positive");
        return false;
      }
      if (position_tolerance_ <= 0 || velocity_tolerance_ <= 0) {
        RCLCPP_ERROR(this->get_logger(), "tolerances must be positive");
        return false;
      }
      if (state_machine_hz_ <= 0 || state_machine_hz_ > 1000) {
        RCLCPP_ERROR(this->get_logger(), "state_machine_hz out of range: %.1f", state_machine_hz_);
        return false;
      }
      if (cave_waypoints_x_.empty() ||
          cave_waypoints_x_.size() != cave_waypoints_y_.size() ||
          cave_waypoints_x_.size() != cave_waypoints_z_.size()) {
        RCLCPP_ERROR(this->get_logger(), "cave_entrance_waypoints invalid or empty");
        return false;
      }
      return true;

    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Parameter error: %s", e.what());
      return false;
    }
  }

  // ===========================================================================
  // Odometry callback — updates current position, velocity, yaw
  // ===========================================================================
  void onCurrentState(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ << msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z;

    current_velocity_ << msg->twist.twist.linear.x,
                         msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z;

    current_yaw_ = tf2::getYaw(msg->pose.pose.orientation);

    if (!received_current_state_) {
      desired_yaw_ = 0.0;  // Fixed yaw for stability during takeoff
    }
    received_current_state_ = true;
  }

  // ===========================================================================
  // Trajectory-complete callback — signals that the planner finished its path
  // ===========================================================================
  void onTrajectoryComplete(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    if (current_state_ == MissionState::EXPLORE_CAVE) {
      return;  // Ignore during exploration (frontier detector manages its own goals)
    }
    RCLCPP_INFO(this->get_logger(), "Trajectory complete signal received");
    trajectory_completed_ = true;
  }

  // ===========================================================================
  // Desired-state publisher — sends position/velocity/acceleration + yaw
  // ===========================================================================
  void publishDesiredState(const Eigen::Vector3d& position,
                           const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d& acceleration = Eigen::Vector3d::Zero(),
                           double yaw = 0.0) {
    auto msg = trajectory_msgs::msg::MultiDOFJointTrajectoryPoint();

    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();
    msg.transforms.push_back(transform);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = velocity.x();
    twist.linear.y = velocity.y();
    twist.linear.z = velocity.z();
    msg.velocities.push_back(twist);

    geometry_msgs::msg::Twist accel;
    accel.linear.x = acceleration.x();
    accel.linear.y = acceleration.y();
    accel.linear.z = acceleration.z();
    msg.accelerations.push_back(accel);

    desired_state_pub_->publish(msg);
  }

  // ===========================================================================
  // Convenience helpers
  // ===========================================================================
  bool isAtPosition(const Eigen::Vector3d& target, double tol = -1.0) {
    if (tol < 0) tol = position_tolerance_;
    return (current_position_ - target).norm() < tol;
  }

  bool isVelocityLow(double tol = -1.0) {
    if (tol < 0) tol = velocity_tolerance_;
    return current_velocity_.norm() < tol;
  }

  // ===========================================================================
  // State transitions — resets state-specific flags
  // ===========================================================================
  void transitionToState(MissionState new_state) {
    if (current_state_ == new_state) return;

    RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
                stateToString(current_state_).c_str(),
                stateToString(new_state).c_str());

    current_state_    = new_state;
    state_start_time_ = this->now();

    if (new_state == MissionState::NAVIGATE_TO_CAVE) {
      navigation_triggered_ = false;
      trajectory_completed_ = false;
    }
    if (new_state == MissionState::EXPLORE_CAVE) {
      exploration_triggered_ = false;
      trajectory_completed_  = false;
    }
  }

  // ===========================================================================
  // Main state machine loop (called at state_machine_hz)
  // ===========================================================================
  void stateMachineLoop() {
    // Broadcast the current state name so other nodes can react
    std_msgs::msg::String state_msg;
    state_msg.data = stateToString(current_state_);
    state_pub_->publish(state_msg);

    if (!received_current_state_) return;  // Wait for first odometry

    switch (current_state_) {
      case MissionState::IDLE:              handleIdle();             break;
      case MissionState::TAKEOFF:           handleTakeoff();          break;
      case MissionState::NAVIGATE_TO_CAVE:  handleNavigateToCave();   break;
      case MissionState::EXPLORE_CAVE:      handleExploreCave();      break;
      case MissionState::FINISHED:          handleFinished();         break;
      default: break;
    }
  }

  // ---------------------------------------------------------------------------
  // IDLE — hold current position, wait idle_duration seconds, then take off
  // ---------------------------------------------------------------------------
  void handleIdle() {
    publishDesiredState(current_position_, Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(), desired_yaw_);

    double elapsed = (this->now() - idle_start_time_).seconds();
    if (elapsed >= idle_duration_) {
      RCLCPP_INFO(this->get_logger(), "IDLE complete (%.1f s). Starting takeoff.", elapsed);
      transitionToState(MissionState::TAKEOFF);
    }
  }

  // ---------------------------------------------------------------------------
  // TAKEOFF — climb to takeoff_altitude while holding x, y position
  // ---------------------------------------------------------------------------
  void handleTakeoff() {
    Eigen::Vector3d target(current_position_.x(), current_position_.y(), takeoff_altitude_);
    publishDesiredState(target, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), desired_yaw_);

    if (isAtPosition(target) && isVelocityLow()) {
      RCLCPP_INFO(this->get_logger(), "Takeoff complete at %.2f m. Navigating to cave.", current_position_.z());
      transitionToState(MissionState::NAVIGATE_TO_CAVE);
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TAKEOFF: alt=%.2f / %.2f m", current_position_.z(), takeoff_altitude_);
    }
  }

  // ---------------------------------------------------------------------------
  // NAVIGATE_TO_CAVE — send waypoints to the trajectory planner, wait for
  // the trajectory_complete signal
  // ---------------------------------------------------------------------------
  void handleNavigateToCave() {
    if (!navigation_triggered_) {
      // Wait for the planner service to become available
      if (!start_navigation_client_->wait_for_service(1s)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "start_navigation service not available, waiting...");
        return;
      }

      // Build the waypoint list from config parameters
      auto request = std::make_shared<mission_interfaces::srv::ExecuteTrajectory::Request>();
      geometry_msgs::msg::PoseArray waypoints;

      for (size_t i = 0; i < cave_waypoints_x_.size(); ++i) {
        geometry_msgs::msg::Pose p;
        p.position.x = cave_waypoints_x_[i];
        p.position.y = cave_waypoints_y_[i];
        p.position.z = cave_waypoints_z_[i];

        // Orientation: face -X direction (yaw = π) so the drone enters the cave
        tf2::Quaternion q;
        q.setRPY(0, 0, M_PI);
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();

        waypoints.poses.push_back(p);
      }
      request->waypoints = waypoints;

      // Send asynchronous service request
      start_navigation_client_->async_send_request(request,
        [this](rclcpp::Client<mission_interfaces::srv::ExecuteTrajectory>::SharedFuture future) {
          auto result = future.get();
          if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Trajectory planner accepted waypoints.");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Trajectory planner rejected: %s", result->message.c_str());
          }
        });

      navigation_triggered_ = true;
      RCLCPP_INFO(this->get_logger(), "Sent %zu waypoints to trajectory planner.", waypoints.poses.size());
    }

    // Wait for the trajectory_complete signal from the planner
    if (trajectory_completed_) {
      RCLCPP_INFO(this->get_logger(),
        "Cave entrance reached [%.1f, %.1f, %.1f]. Starting exploration.",
        current_position_.x(), current_position_.y(), current_position_.z());
      transitionToState(MissionState::EXPLORE_CAVE);
    }
  }

  // ---------------------------------------------------------------------------
  // EXPLORE_CAVE — delegate control to the frontier-based explorer
  // ---------------------------------------------------------------------------
  void handleExploreCave() {
    if (!exploration_triggered_) {
      if (!start_exploration_client_->wait_for_service(1s)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "start_exploration service not available, waiting...");
        return;
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      start_exploration_client_->async_send_request(request);

      exploration_triggered_ = true;
      RCLCPP_INFO(this->get_logger(), "Exploration trigger sent to frontier detector.");
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "EXPLORE_CAVE: exploration in progress.");
  }

  // ---------------------------------------------------------------------------
  // FINISHED — hold current position
  // ---------------------------------------------------------------------------
  void handleFinished() {
    publishDesiredState(current_position_, Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(), desired_yaw_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Mission FINISHED. Holding at [%.1f, %.1f, %.1f]",
      current_position_.x(), current_position_.y(), current_position_.z());
  }
};

// =============================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionControlNode>());
  rclcpp::shutdown();
  return 0;
}
