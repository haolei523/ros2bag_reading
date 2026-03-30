#include "bag_reading/bag_reading.hpp"

PlaybackNode::PlaybackNode()
: Node("playback_node")
{
  // publisher_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/pose", 10);
  // timer_ = this->create_wall_timer(
  //     100ms, std::bind(&PlaybackNode::timer_callback, this));

  // 读取配置文件信息，初始化变量
  std::string bag_filepath = this->declare_parameter<std::string>(
    "bag_filepath", "/home");
  image_sub_topic = this->declare_parameter<std::string>(
    "topic.image", "/camera");
  lidar_sub_topic = this->declare_parameter<std::string>(
    "topic.lidar", "/lidar");
  image_output_path = this->declare_parameter<std::string>(
    "output_path.image", "/home");
  lidar_output_path = this->declare_parameter<std::string>(
    "output_path.lidar", "/home");
  debug = this->declare_parameter<bool>(
    "debug", false);
  sync = this->declare_parameter<bool>(
    "sync", false);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_filepath;
  reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader_->open(storage_options);
  lidar_stamp = -10.0;
  camera_stamp = -15.0;
  index = 0;
}

template<typename S>
void PlaybackNode::write_image(S data, double stamp) {
  cv_bridge::CvImagePtr cv_image;
  cv_image = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
  
  if (debug)
  {
    std::cout << "Ready write image: " << image_output_path << "/" << stamp << ".png" << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Ready write image: %s", image_output_path, camera_stamp);
  }
  cv::imwrite(image_output_path+"/"+std::to_string(stamp)+".png", cv_image->image);
}

template<typename S>
void PlaybackNode::write_pointcloud(S data, double stamp) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(data, *pc);
  pcl::io::savePCDFile(lidar_output_path+"/"+std::to_string(stamp)+".pcd", *pc);
  if (debug)
  {
    std::cout << "Ready write pcd: " << lidar_output_path << "/" << stamp << ".png" << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Ready write image: %s", image_output_path, camera_stamp);
  }
}

template<typename SP, typename S>
void PlaybackNode::save(rclcpp::SerializedMessage serialized_msg, std::string topic) {
  SP ros_msg = std::make_shared<S>();
  rclcpp::Serialization<S> serialization_;
  serialization_.deserialize_message(&serialized_msg, ros_msg.get());
  S data = *ros_msg.get();
  double stamp = ros_msg->header.stamp.sec * 1e3 + ros_msg->header.stamp.nanosec * 1e-6;

  // if (topic == image_sub_topic) 
  if constexpr (std::is_same_v<S, sensor_msgs::msg::Image>)
  {
    write_image(data, stamp);
  }
    

  // if (topic == lidar_sub_topic) 
  if constexpr (std::is_same_v<S, sensor_msgs::msg::PointCloud2>)
  {
    write_pointcloud(data, stamp);
  }
    
}

/**
 * 转换过程，初始化ros结束后开始执行转换
 * 1、 读取rosbag文件
 * 2、 按顺序将每帧数据转换
 */
void PlaybackNode::timer_callback()
{
  std::deque<rclcpp::SerializedMessage> camera_container;
  std::deque<rclcpp::SerializedMessage> lidar_container;

  while (reader_->has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();
    
    // 只处理rosbag中的图像或点云数据
    if (msg->topic_name == image_sub_topic || msg->topic_name == lidar_sub_topic) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      if (sync) {
        if (msg->topic_name == image_sub_topic) camera_container.push_back(serialized_msg);
        if (msg->topic_name == lidar_sub_topic) lidar_container.push_back(serialized_msg);

        if (lidar_container.empty()) continue;
        sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        lidar_serialization_.deserialize_message(&lidar_container.front(), lidar_msg.get());
        lidar_stamp = lidar_msg->header.stamp.sec * 1e3 + lidar_msg->header.stamp.nanosec * 1e-9;

        while (!camera_container.empty() && !lidar_container.empty()) {
          sensor_msgs::msg::Image::SharedPtr camera_msg = std::make_shared<sensor_msgs::msg::Image>();
          camera_serialization_.deserialize_message(&camera_container.front(), camera_msg.get());
          camera_stamp = camera_msg->header.stamp.sec * 1e3 + camera_msg->header.stamp.nanosec * 1e-9;
          std::cout << "delta stamp: " << camera_stamp - lidar_stamp << std::endl;
          if (camera_stamp - lidar_stamp > 0.1) {
            lidar_container.pop_front();
            break;
          } else if (camera_stamp - lidar_stamp < -0.1) {
            camera_container.pop_front();
            continue;
          } else {
            save<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image>(camera_container.front(), image_sub_topic);
            save<sensor_msgs::msg::PointCloud2::SharedPtr, sensor_msgs::msg::PointCloud2>(lidar_container.front(), lidar_sub_topic);
            camera_container.pop_front();
            lidar_container.pop_front();
          }
        }
      } else {
        // 转换图像数据为png
        if (msg->topic_name == image_sub_topic) {
          save<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image>(serialized_msg, image_sub_topic);
        }

        // 转换lidar数据为pcd
        if (msg->topic_name == lidar_sub_topic) {
          save<sensor_msgs::msg::PointCloud2::SharedPtr, sensor_msgs::msg::PointCloud2>(serialized_msg, lidar_sub_topic);
        }
      }
      
      


      
      
    }
  }
}


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  PlaybackNode pn;
  pn.timer_callback();  // 执行转换
  // rclcpp::spin(std::make_shared<PlaybackNode>());
  rclcpp::shutdown();

  return 0;
}
