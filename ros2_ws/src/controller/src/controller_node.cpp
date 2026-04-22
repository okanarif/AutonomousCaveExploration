// =============================================================================
// Geometric Tracking Controller for Quadrotor UAV
// =============================================================================
//
// Implements the SE(3) geometric tracking controller from:
//   Lee, Taeyoung, Melvin Leok, N. Harris McClamroch.
//   "Geometric tracking control of a quadrotor UAV on SE(3)."
//   49th IEEE Conference on Decision and Control (CDC), 2010.
//
// The controller computes desired rotor speeds from position/velocity errors
// and orientation errors on SO(3), producing wrench commands that are mapped
// to individual rotor speeds via the wrench-to-rotor allocation matrix.
//
// Frame convention: Z-axis points UP (differs from the paper where Z is down).
// This flips the sign of gravity and thrust terms compared to the original
// equations. See inline comments for details.
// =============================================================================

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <Eigen/Dense>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PI M_PI

class ControllerNode : public rclcpp::Node {
  // ---------------------------------------------------------------------------
  // ROS interfaces
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr rotor_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ---------------------------------------------------------------------------
  // Controller gains – [1] eq. (15), (16)
  // ---------------------------------------------------------------------------
  double kx, kv, kr, komega;

  // ---------------------------------------------------------------------------
  // Physical constants
  // ---------------------------------------------------------------------------
  double m;              // Mass of the UAV [kg]
  double g;              // Gravitational acceleration [m/s^2]
  double d;              // Arm length: center-of-mass to propeller [m]
  double cf;             // Propeller lift coefficient: thrust = cf * w^2
  double cd;             // Propeller drag coefficient: torque = cd * w^2
  Eigen::Matrix3d J;     // Diagonal inertia matrix [kg*m^2]
  Eigen::Vector3d e3;    // Unit vector along Z: [0, 0, 1]
  Eigen::MatrixXd F2W;   // Wrench-to-rotor-speed mapping matrix (4x4)

  // ---------------------------------------------------------------------------
  // Current state (updated by odometry callback)
  // ---------------------------------------------------------------------------
  Eigen::Vector3d x;     // Position in world frame [m]
  Eigen::Vector3d v;     // Velocity in world frame [m/s]
  Eigen::Matrix3d R;     // Orientation (rotation matrix, body -> world)
  Eigen::Vector3d omega; // Angular velocity in body frame [rad/s]

  // ---------------------------------------------------------------------------
  // Desired state (updated by trajectory callback)
  // ---------------------------------------------------------------------------
  Eigen::Vector3d xd;    // Desired position [m]
  Eigen::Vector3d vd;    // Desired velocity [m/s]
  Eigen::Vector3d ad;    // Desired acceleration [m/s^2]
  double yawd;           // Desired yaw angle [rad]

  double hz;             // Control loop frequency [Hz]

  bool received_desired_state_;
  bool received_current_state_;

  // ---------------------------------------------------------------------------
  // Helper: extract the "vee" map (inverse of skew-symmetric hat map)
  // ---------------------------------------------------------------------------
  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in) {
    Eigen::Vector3d out;
    out << in(2, 1), in(0, 2), in(1, 0);
    return out;
  }

  // ---------------------------------------------------------------------------
  // Helper: signed square root (preserves sign for negative squared speeds)
  // ---------------------------------------------------------------------------
  static double signed_sqrt(double val) {
    return val > 0 ? sqrt(val) : -sqrt(-val);
  }

public:
  ControllerNode()
  : rclcpp::Node("controller_node"),
    e3(0, 0, 1),
    F2W(4, 4),
    received_desired_state_(false),
    received_current_state_(false),
    x(0, 0, 0),
    v(0, 0, 0),
    R(Eigen::Matrix3d::Identity()),
    omega(0, 0, 0)
  {
    // Declare parameters with default values (overridden by YAML config)
    this->declare_parameter<double>("kx", 12.0);
    this->declare_parameter<double>("kv", 5.0);
    this->declare_parameter<double>("kr", 8.5);
    this->declare_parameter<double>("komega", 1.1);
    this->declare_parameter<double>("control_loop_hz", 1000.0);
    this->declare_parameter<double>("mass", 1.0);
    this->declare_parameter<double>("gravity", 9.81);
    this->declare_parameter<double>("arm_length", 0.3);
    this->declare_parameter<double>("lift_coefficient", 1e-3);
    this->declare_parameter<double>("drag_coefficient", 1e-5);
    this->declare_parameter<double>("inertia_xx", 0.01);
    this->declare_parameter<double>("inertia_yy", 0.01);
    this->declare_parameter<double>("inertia_zz", 0.02);
    this->declare_parameter<std::string>("desired_state_topic", "desired_state");
    this->declare_parameter<std::string>("current_state_topic", "current_state_est");
    this->declare_parameter<std::string>("rotor_speed_cmd_topic", "rotor_speed_cmds");
    this->declare_parameter<int>("subscriber_queue_size", 10);
    this->declare_parameter<int>("publisher_queue_size", 10);

    // Load and validate all parameters
    if (!loadParameters()) {
      RCLCPP_FATAL(this->get_logger(),
        "Failed to load required parameters. Node cannot start.");
      rclcpp::shutdown();
      return;
    }

    // Resolve topic names and queue sizes
    int sub_queue_size = this->get_parameter("subscriber_queue_size").as_int();
    int pub_queue_size = this->get_parameter("publisher_queue_size").as_int();
    std::string desired_state_topic = this->get_parameter("desired_state_topic").as_string();
    std::string current_state_topic = this->get_parameter("current_state_topic").as_string();
    std::string rotor_cmd_topic = this->get_parameter("rotor_speed_cmd_topic").as_string();

    // Set up subscribers
    desired_state_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      desired_state_topic, sub_queue_size,
      std::bind(&ControllerNode::onDesiredState, this, std::placeholders::_1));

    current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      current_state_topic, sub_queue_size,
      std::bind(&ControllerNode::onCurrentState, this, std::placeholders::_1));

    // Set up publisher
    rotor_pub_ = this->create_publisher<mav_msgs::msg::Actuators>(rotor_cmd_topic, pub_queue_size);

    // Start the control loop timer
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&ControllerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Controller node initialized (%.0f Hz)", hz);
  }

private:
  // ---------------------------------------------------------------------------
  // Load all parameters from the parameter server and validate them
  // ---------------------------------------------------------------------------
  bool loadParameters() {
    try {
      // Controller gains
      kx     = this->get_parameter("kx").as_double();
      kv     = this->get_parameter("kv").as_double();
      kr     = this->get_parameter("kr").as_double();
      komega = this->get_parameter("komega").as_double();

      // Control loop frequency
      hz = this->get_parameter("control_loop_hz").as_double();

      // Physical constants
      m  = this->get_parameter("mass").as_double();
      g  = this->get_parameter("gravity").as_double();
      d  = this->get_parameter("arm_length").as_double();
      cf = this->get_parameter("lift_coefficient").as_double();
      cd = this->get_parameter("drag_coefficient").as_double();

      // Diagonal inertia matrix
      double Jxx = this->get_parameter("inertia_xx").as_double();
      double Jyy = this->get_parameter("inertia_yy").as_double();
      double Jzz = this->get_parameter("inertia_zz").as_double();
      J << Jxx, 0.0, 0.0,
           0.0, Jyy, 0.0,
           0.0, 0.0, Jzz;

      // Validate
      if (kx <= 0 || kv <= 0 || kr <= 0 || komega <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Controller gains must be positive!");
        return false;
      }
      if (hz <= 0 || hz > 10000) {
        RCLCPP_ERROR(this->get_logger(), "Invalid control loop frequency: %.1f Hz", hz);
        return false;
      }
      if (m <= 0 || g <= 0 || d <= 0 || cf <= 0 || cd <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Physical constants must be positive!");
        return false;
      }
      if (Jxx <= 0 || Jyy <= 0 || Jzz <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Inertia values must be positive!");
        return false;
      }

      return true;

    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
      RCLCPP_FATAL(this->get_logger(), "Parameter not declared: %s", e.what());
      return false;
    } catch (const rclcpp::ParameterTypeException& e) {
      RCLCPP_FATAL(this->get_logger(), "Parameter type mismatch: %s", e.what());
      return false;
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Error loading parameters: %s", e.what());
      return false;
    }
  }

  // ---------------------------------------------------------------------------
  // Callback: desired state (position, velocity, acceleration, yaw)
  // ---------------------------------------------------------------------------
  void onDesiredState(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr msg) {
    // Extract desired position
    xd << msg->transforms[0].translation.x,
          msg->transforms[0].translation.y,
          msg->transforms[0].translation.z;

    // Extract desired velocity
    vd << msg->velocities[0].linear.x,
          msg->velocities[0].linear.y,
          msg->velocities[0].linear.z;

    // Extract desired acceleration
    ad << msg->accelerations[0].linear.x,
          msg->accelerations[0].linear.y,
          msg->accelerations[0].linear.z;

    // Extract desired yaw from quaternion
    yawd = tf2::getYaw(msg->transforms[0].rotation);

    received_desired_state_ = true;
  }

  // ---------------------------------------------------------------------------
  // Callback: current state (odometry -> position, velocity, orientation)
  // ---------------------------------------------------------------------------
  void onCurrentState(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Current position
    x << msg->pose.pose.position.x,
         msg->pose.pose.position.y,
         msg->pose.pose.position.z;

    // Current velocity
    v << msg->twist.twist.linear.x,
         msg->twist.twist.linear.y,
         msg->twist.twist.linear.z;

    // Current orientation: quaternion -> rotation matrix
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    R = q.toRotationMatrix();

    // Angular velocity: convert from world frame to body frame
    Eigen::Vector3d omega_world;
    omega_world << msg->twist.twist.angular.x,
                   msg->twist.twist.angular.y,
                   msg->twist.twist.angular.z;
    omega = R.transpose() * omega_world;

    received_current_state_ = true;
  }

  // ---------------------------------------------------------------------------
  // Main control loop: geometric SE(3) controller
  // ---------------------------------------------------------------------------
  void controlLoop() {
    // Wait until both desired and current states are available
    if (!received_desired_state_ || !received_current_state_) {
      return;
    }

    // --- Position and velocity errors – [1] eq. (6), (7) ---
    Eigen::Vector3d ex = x - xd;
    Eigen::Vector3d ev = v - vd;

    // --- Desired rotation matrix Rd – [1] eq. (12) ---
    // Note: Z-axis UP convention flips the gravity sign vs. the paper.
    Eigen::Vector3d b3d = (-kx * ex - kv * ev + m * g * e3 + m * ad);
    b3d.normalize();

    Eigen::Vector3d b1c(cos(yawd), sin(yawd), 0);  // Heading direction from desired yaw
    Eigen::Vector3d b2d = b3d.cross(b1c);
    b2d.normalize();

    Eigen::Vector3d b1d = b2d.cross(b3d);
    b1d.normalize();

    Eigen::Matrix3d Rd;
    Rd << b1d, b2d, b3d;  // Columns: [b1d | b2d | b3d]

    // --- Orientation and angular velocity errors – [1] eq. (10), (11) ---
    Eigen::Matrix3d eR_matrix = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
    Eigen::Vector3d er = Vee(eR_matrix);
    Eigen::Vector3d eomega = -omega;  // Desired angular velocity assumed zero

    // --- Desired wrench (total thrust + torques) – [1] eq. (15), (16) ---
    double f = (-kx * ex - kv * ev + m * g * e3 + m * ad).dot(R * e3);
    Eigen::Vector3d M = -kr * er - komega * eomega + omega.cross(J * omega);

    Eigen::Vector4d wrench;
    wrench << f, M(0), M(1), M(2);

    // --- Wrench-to-rotor allocation ---
    // Our frame has a 45° offset from the paper's arm convention
    // (see lecture notes eq. 6.9).
    double s = sqrt(2.0) / 2.0;  // sin(45°) = cos(45°)

    F2W << cf,          cf,          cf,          cf,
           cf*d*s,      cf*d*s,     -cf*d*s,     -cf*d*s,
          -cf*d*s,      cf*d*s,      cf*d*s,     -cf*d*s,
           cd,         -cd,          cd,          -cd;

    // Solve for squared rotor speeds: F2W * w^2 = wrench
    Eigen::Vector4d w_squared = F2W.colPivHouseholderQr().solve(wrench);

    // Recover signed rotor speeds
    double w1 = signed_sqrt(w_squared(0));
    double w2 = signed_sqrt(w_squared(1));
    double w3 = signed_sqrt(w_squared(2));
    double w4 = signed_sqrt(w_squared(3));

    // --- Publish rotor speed command ---
    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.resize(4);
    cmd.angular_velocities[0] = w1;
    cmd.angular_velocities[1] = w2;
    cmd.angular_velocities[2] = w3;
    cmd.angular_velocities[3] = w4;

    rotor_pub_->publish(cmd);
  }
};

// =============================================================================
// Entry point
// =============================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
