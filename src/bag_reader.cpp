#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

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
    PlaybackNode()
    : Node("playback_node")
    {
      // publisher_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/pose", 10);
      // timer_ = this->create_wall_timer(
      //     100ms, std::bind(&PlaybackNode::timer_callback, this));
      std::string bag_filepath = this->declare_parameter<std::string>(
        "bag_filepath", "/home");
      image_sub_topic = this->declare_parameter<std::string>(
        "topic.image", "/zw040201/lq_camera");
      lidar_sub_topic = this->declare_parameter<std::string>(
        "topic.lidar", "/zw040201/lq_lidar_pointcloud");
      image_output_path = this->declare_parameter<std::string>(
        "output_path.image", "/home");
      lidar_output_path = this->declare_parameter<std::string>(
        "output_path.lidar", "/home");
      debug = this->declare_parameter<bool>(
        "debug", false);

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_filepath;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);
      lidar_stamp = -10.0;
      camera_stamp = -15.0;
      index = 0;
    }

    void timer_callback()
    {
      while (reader_->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();
        
        if (msg->topic_name == image_sub_topic || msg->topic_name == lidar_sub_topic) {
          rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
          
          if (msg->topic_name == image_sub_topic) {
            sensor_msgs::msg::Image::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Image>();
            camera_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
            camera = *ros_msg.get();
            cv_bridge::CvImagePtr cv_image;
            cv_image = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8);
            camera_stamp = ros_msg->header.stamp.sec * 1e3 + ros_msg->header.stamp.nanosec * 1e-6;
            if (debug)
            {
              std::cout << "Ready write image: " << image_output_path << "/" << index << ".png" << std::endl;
              // RCLCPP_INFO(this->get_logger(), "Ready write image: %s", image_output_path, camera_stamp);
            }
              
            cv::imwrite(image_output_path+"/"+std::to_string(index)+".png", cv_image->image);
            index ++;
          }
          if (msg->topic_name == lidar_sub_topic) {
            sensor_msgs::msg::PointCloud2::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            lidar_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
            lidar = *ros_msg.get();
            lidar_stamp = ros_msg->header.stamp.sec * 1e3 + ros_msg->header.stamp.nanosec * 1e-6;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(lidar, *pc);
            pcl::io::savePCDFile(lidar_output_path+"/"+std::to_string(lidar_stamp)+".pcd", *pc);
            if (debug)
            {
              std::cout << "Ready write pcd: " << lidar_output_path << "/" << index << ".png" << std::endl;
              // RCLCPP_INFO(this->get_logger(), "Ready write image: %s", image_output_path, camera_stamp);
            }
          }
          
        }
      }
    }

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
};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  PlaybackNode pn;
  pn.timer_callback();
  // rclcpp::spin(std::make_shared<PlaybackNode>());
  rclcpp::shutdown();

  return 0;
}
