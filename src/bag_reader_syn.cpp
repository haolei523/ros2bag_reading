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

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = "/media/haolei/P4/zju_object_detection/rosbag2_2024_08_31-16_37_50_21-27/rosbag2_2024_10_28-07_22_30_0.db3";
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
        
        if (msg->topic_name == "/ivs/vehicle0/visual0/rgb_image" || msg->topic_name == "/ivs/vehicle0/lidar0") {
          rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
          
          
          
          if (msg->topic_name == "/ivs/vehicle0/visual0/rgb_image") {
            // camera_stamp = ros_msg->header.stamp.sec * 1e3 + ros_msg->header.stamp.nsec * 1e-6;
            sensor_msgs::msg::Image::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Image>();
            camera_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
            camera = *ros_msg.get();
          }
          if (msg->topic_name == "/ivs/vehicle0/lidar0") {
            sensor_msgs::msg::PointCloud2::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            lidar_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
            lidar = *ros_msg.get();
          }
          if (std::abs((lidar.header.stamp.sec * 1e3 + lidar.header.stamp.nanosec * 1e-6) - 
              (camera.header.stamp.sec * 1e3 + camera.header.stamp.nanosec * 1e-6)) < 100) {
            // save
            printf("lidar stamp: %15lf \n", lidar.header.stamp.sec * 1e3 + lidar.header.stamp.nanosec * 1e-6);
            printf("camera stamp: %15lf \n", camera.header.stamp.sec * 1e3 + camera.header.stamp.nanosec * 1e-6);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(lidar, *pc);
            pcl::io::savePCDFile("/bag_reading/lidar/"+std::to_string(index)+".pcd", *pc);
            
            cv_bridge::CvImagePtr cv_image;
            cv_image = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8);
            cv::imwrite("/bag_reading/image/"+std::to_string(index)+".png", cv_image->image);
            index ++;
          }
        }
        
        

        // publisher_->publish(*ros_msg);
        // std::cout << '(' << ros_msg->x << ", " << ros_msg->y << ")\n";

        // break;
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
