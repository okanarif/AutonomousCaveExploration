// =============================================================================
// Lantern Detection Node
// =============================================================================
//
// Detects and localises lanterns inside the cave using:
//   1. A semantic segmentation camera (provides binary lantern mask)
//   2. A depth camera (provides 3D distance per pixel)
//   3. TF2 transforms (camera → world frame)
//
// Processing pipeline per semantic image frame:
//   - Extract connected components from the binary mask.
//   - For each component centroid, back-project the pixel to 3D using
//     the depth image and camera intrinsics.
//   - Transform the 3D point from camera frame to world frame via TF2.
//   - Check uniqueness against already-known lanterns using a distance
//     threshold to avoid double-counting.
//   - Publish newly discovered lantern positions and an incrementing count.
//
// All parameters (thresholds, topic names, frame IDs) are loaded from YAML.
// =============================================================================

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <vector>
#include <string>
#include <sstream>

// =============================================================================
class LanternDetector : public rclcpp::Node
{
public:
  LanternDetector()
  : Node("lantern_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    lantern_count_(0)
  {
    // ------------------------------------------------------------------
    // Declare parameters with defaults
    // ------------------------------------------------------------------
    // Detection thresholds
    this->declare_parameter<double>("distance_threshold", 100.0);
    this->declare_parameter<double>("drone_lantern_dist", 30.0);

    // Coordinate frames
    this->declare_parameter<std::string>("camera_frame", "camera");
    this->declare_parameter<std::string>("world_frame",  "world");

    // Subscriber topic names
    this->declare_parameter<std::string>("odom_topic",           "/current_state_est");
    this->declare_parameter<std::string>("semantic_image_topic", "/Quadrotor/Sensors/SemanticCamera/image_raw");
    this->declare_parameter<std::string>("depth_image_topic",    "/realsense/depth/image");
    this->declare_parameter<std::string>("camera_info_topic",    "/realsense/depth/camera_info");

    // Publisher topic names
    this->declare_parameter<std::string>("lantern_positions_topic", "/lantern_positions");
    this->declare_parameter<std::string>("detected_lanterns_topic", "/detected_lanterns");
    this->declare_parameter<std::string>("num_lanterns_topic",      "/num_lanterns");

    // ------------------------------------------------------------------
    // Load parameters
    // ------------------------------------------------------------------
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    drone_lantern_dist_ = this->get_parameter("drone_lantern_dist").as_double();
    camera_frame_       = this->get_parameter("camera_frame").as_string();
    world_frame_        = this->get_parameter("world_frame").as_string();

    std::string odom_topic           = this->get_parameter("odom_topic").as_string();
    std::string semantic_image_topic = this->get_parameter("semantic_image_topic").as_string();
    std::string depth_image_topic    = this->get_parameter("depth_image_topic").as_string();
    std::string camera_info_topic    = this->get_parameter("camera_info_topic").as_string();
    std::string lantern_pos_topic    = this->get_parameter("lantern_positions_topic").as_string();
    std::string detected_txt_topic   = this->get_parameter("detected_lanterns_topic").as_string();
    std::string num_lanterns_topic   = this->get_parameter("num_lanterns_topic").as_string();

    // ------------------------------------------------------------------
    // Subscribers
    // ------------------------------------------------------------------
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(1),
      std::bind(&LanternDetector::odomCallback, this, std::placeholders::_1));

    semantic_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      semantic_image_topic, rclcpp::QoS(10),
      std::bind(&LanternDetector::semanticCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic, rclcpp::QoS(10),
      std::bind(&LanternDetector::depthCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::QoS(10),
      std::bind(&LanternDetector::cameraInfoCallback, this, std::placeholders::_1));

    // ------------------------------------------------------------------
    // Publishers
    // ------------------------------------------------------------------
    lantern_positions_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      lantern_pos_topic, rclcpp::QoS(10));

    lantern_text_pub_ = this->create_publisher<std_msgs::msg::String>(
      detected_txt_topic, rclcpp::QoS(10));

    lantern_count_pub_ = this->create_publisher<std_msgs::msg::Int16>(
      num_lanterns_topic, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Lantern detection node initialized");
  }

private:
  // ---- ROS interfaces -----------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr    semantic_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr    depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lantern_positions_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            lantern_text_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr             lantern_count_pub_;

  // ---- TF2 ----------------------------------------------------------------
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ---- Sensor data ---------------------------------------------------------
  cv::Mat depth_image_;
  sensor_msgs::msg::CameraInfo camera_info_;
  bool has_camera_info_{false};

  // ---- Drone state ---------------------------------------------------------
  geometry_msgs::msg::Point drone_position_;

  // ---- Detection state -----------------------------------------------------
  std::vector<geometry_msgs::msg::Point> known_lantern_positions_;
  int lantern_count_;

  // ---- Parameters ----------------------------------------------------------
  double distance_threshold_;   // Min distance between two distinct lanterns
  double drone_lantern_dist_;   // Max detection range from drone to lantern
  std::string camera_frame_;
  std::string world_frame_;

  // ===========================================================================
  // Callbacks
  // ===========================================================================

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    drone_position_ = msg->pose.pose.position;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr depth_msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      depth_image_ = cv_ptr->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge error: %s", e.what());
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info_msg) {
    camera_info_     = *info_msg;
    has_camera_info_ = true;
  }

  // ===========================================================================
  // Semantic camera callback — main detection pipeline
  // ===========================================================================
  void semanticCallback(const sensor_msgs::msg::Image::SharedPtr semantic_msg) {
    // Convert semantic mask to OpenCV grayscale
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(semantic_msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Semantic cv_bridge error: %s", e.what());
      return;
    }

    cv::Mat semantic_img = cv_ptr->image;
    if (cv::countNonZero(semantic_img) == 0) return;  // No lantern pixels in frame

    // Find connected components (individual lantern blobs)
    cv::Mat labels, stats, centroids;
    int num_components = cv::connectedComponentsWithStats(
      semantic_img, labels, stats, centroids, 8, CV_32S);

    std_msgs::msg::Float32MultiArray new_lanterns;

    for (int i = 1; i < num_components; ++i) {
      int px = static_cast<int>(centroids.at<double>(i, 0));
      int py = static_cast<int>(centroids.at<double>(i, 1));

      // Back-project pixel to 3D point in camera frame
      geometry_msgs::msg::Point cam_point = pixelTo3D(px, py);

      // Skip detections with invalid depth
      if (depth_image_.empty() ||
          px < 0 || px >= depth_image_.cols ||
          py < 0 || py >= depth_image_.rows) continue;

      uint16_t depth_mm = depth_image_.at<uint16_t>(py, px);
      if (depth_mm == 0) continue;

      // Transform from camera frame to world frame
      geometry_msgs::msg::PointStamped world_point = transformToWorldFrame(cam_point);

      // Filter by drone-to-lantern distance
      double dist = distanceBetween(drone_position_, world_point.point);
      if (dist > drone_lantern_dist_) continue;

      // Check if this is a previously unseen lantern
      if (!isNewLantern(world_point.point)) continue;

      lantern_count_++;
      known_lantern_positions_.push_back(world_point.point);

      RCLCPP_INFO(this->get_logger(),
        "Lantern #%d at (%.1f, %.1f, %.1f), dist=%.1f m",
        lantern_count_,
        world_point.point.x, world_point.point.y, world_point.point.z, dist);

      // Append to the positions array
      new_lanterns.data.push_back(static_cast<float>(world_point.point.x));
      new_lanterns.data.push_back(static_cast<float>(world_point.point.y));
      new_lanterns.data.push_back(static_cast<float>(world_point.point.z));

      // Publish human-readable text
      std_msgs::msg::String text_msg;
      std::stringstream ss;
      ss << "Lantern " << lantern_count_
         << " at (" << world_point.point.x
         << ", " << world_point.point.y
         << ", " << world_point.point.z
         << "), dist=" << dist << " m";
      text_msg.data = ss.str();
      lantern_text_pub_->publish(text_msg);

      // Publish updated count
      std_msgs::msg::Int16 count_msg;
      count_msg.data = static_cast<int16_t>(lantern_count_);
      lantern_count_pub_->publish(count_msg);
    }

    if (!new_lanterns.data.empty()) {
      lantern_positions_pub_->publish(new_lanterns);
    }
  }

  // ===========================================================================
  // Back-project a pixel (px, py) to a 3D point in camera frame using depth
  // ===========================================================================
  geometry_msgs::msg::Point pixelTo3D(int px, int py) {
    geometry_msgs::msg::Point pt;

    if (depth_image_.empty() || !has_camera_info_ || camera_info_.k.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Depth image or camera info not yet available.");
      return pt;
    }

    if (px < 0 || py < 0 || px >= depth_image_.cols || py >= depth_image_.rows) return pt;

    float depth_m = depth_image_.at<uint16_t>(py, px) * 0.001f;

    float fx = static_cast<float>(camera_info_.k[0]);
    float fy = static_cast<float>(camera_info_.k[4]);
    float cx = static_cast<float>(camera_info_.k[2]);
    float cy = static_cast<float>(camera_info_.k[5]);

    pt.x = (static_cast<float>(px) - cx) * depth_m / fx;
    pt.y = (static_cast<float>(py) - cy) * depth_m / fy;
    pt.z = depth_m;
    return pt;
  }

  // ===========================================================================
  // Transform a point from camera frame to world frame via TF2
  // ===========================================================================
  geometry_msgs::msg::PointStamped transformToWorldFrame(
      const geometry_msgs::msg::Point& cam_point)
  {
    geometry_msgs::msg::PointStamped camera_pt, world_pt;
    camera_pt.header.frame_id = camera_frame_;
    camera_pt.header.stamp    = this->now();
    camera_pt.point           = cam_point;

    try {
      world_pt = tf_buffer_.transform(camera_pt, world_frame_, tf2::durationFromSec(0.2));
      return world_pt;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform error: %s", ex.what());
      return camera_pt;
    }
  }

  // ===========================================================================
  // Utility functions
  // ===========================================================================

  // Check if the candidate lantern is far enough from all known lanterns
  bool isNewLantern(const geometry_msgs::msg::Point& candidate) {
    for (const auto& known : known_lantern_positions_) {
      if (distanceBetween(known, candidate) < distance_threshold_) return false;
    }
    return true;
  }

  double distanceBetween(const geometry_msgs::msg::Point& p1,
                         const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }
};

// =============================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LanternDetector>());
  rclcpp::shutdown();
  return 0;
}
