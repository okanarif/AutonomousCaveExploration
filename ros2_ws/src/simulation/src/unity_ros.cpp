#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <thread>
#include <chrono>
#include "unity_stream_parser.h"
#include "rgb_camera_parser.h"
#include "depth_camera_parser.h"
#include "fisheye_camera_parser.h"
#include "imu_parser.h"
#include "true_state_parser.h"
#include "unity_command_stream.h"


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("unity_ros");

  RCLCPP_INFO(node->get_logger(), "Starting TCPStreamReader");
  
  std::unique_ptr<TCPStreamReader> stream_reader;
  std::unique_ptr<UnityCommandStream> command_stream;

  bool connected = false;
  while (rclcpp::ok() && !connected) {
    try {
      RCLCPP_INFO(node->get_logger(), "Connecting to Unity on ports 9998/9999...");
      stream_reader = std::make_unique<TCPStreamReader>("127.0.0.1", "9998");
      stream_reader->WaitConnect();
      command_stream = std::make_unique<UnityCommandStream>("127.0.0.1", "9999");
      connected = true;
      RCLCPP_INFO(node->get_logger(), "Bridge connected successfully.");
    } catch (const libsocket::socket_exception& e) {
      RCLCPP_WARN(node->get_logger(), "Connection failed (%s). Retrying in 1s...", e.mesg.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  if (!rclcpp::ok()) return 1;

  IMUParser imu_parser;

  auto clock_pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  double last_monotonic_timestamp = -1.0;
  double last_clock_published = -1.0;

  std::vector<std::shared_ptr<UnityStreamParser>> stream_parsers(UnityMessageType::MESSAGE_TYPE_COUNT);

  stream_parsers[UnityMessageType::UNITY_STATE] = std::make_shared<TrueStateParser>();
  stream_parsers[UnityMessageType::UNITY_IMU] = std::make_shared<IMUParser>();
  stream_parsers[UnityMessageType::UNITY_CAMERA] = std::make_shared<RGBCameraParser>();
  stream_parsers[UnityMessageType::UNITY_DEPTH] = std::make_shared<DepthCameraParser>();
  stream_parsers[UnityMessageType::UNITY_FISHEYE] = std::make_shared<FisheyeCameraParser>();
  
  while (stream_reader->Good() && rclcpp::ok()) {    
    uint32_t magic = stream_reader->ReadUInt();


    if(magic == 0xDEADC0DE) {
      double ros_time = node->now().seconds();
      UnityHeader header;
      header.type = static_cast<UnityMessageType>(stream_reader->ReadUInt());
      uint64_t timestamp_raw = stream_reader->ReadUInt64();
      header.timestamp = static_cast<double>(timestamp_raw) * 1e-7;
      header.name = stream_reader->ReadString();
      
      if(header.type < UnityMessageType::MESSAGE_TYPE_COUNT) {
        stream_parsers[header.type]->ParseMessage(header, *stream_reader);
      }

      // Publish simulation clock if timestamp actually moved forward
      if (header.timestamp > last_monotonic_timestamp) {
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = rclcpp::Time(static_cast<int64_t>(header.timestamp * 1e9));
        clock_pub->publish(clock_msg);
        last_monotonic_timestamp = header.timestamp;
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Stream corrupted, could not parse unity message");
    }

    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
