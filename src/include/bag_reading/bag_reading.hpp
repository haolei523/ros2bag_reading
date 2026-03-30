#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
public:
  PlaybackNode();
  void timer_callback();

  template<typename SP, typename S>
  void save(rclcpp::SerializedMessage serialized_msg, std::string topic);

  template<typename S>
  void write_image(S data, double stamp);
  template<typename S>
  void write_pointcloud(S data, double stamp);

private:
  //rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;
  rclcpp::Serialization<sensor_msgs::msg::Image> camera_serialization_;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> lidar_serialization_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;

  double lidar_stamp;
  double camera_stamp;

  sensor_msgs::msg::PointCloud2 lidar;
  sensor_msgs::msg::Image camera;

  int index;

  std::string image_sub_topic;
  std::string lidar_sub_topic;
  std::string image_output_path;
  std::string lidar_output_path;

  bool debug;
  bool sync;
};