# ros2 bag 转换 pcd
### 采集激光雷达数据
``` bash
lidar = /lidar  # 改为激光雷达对应ros2话题名称
ros2 bag record $lidar
```
### 环境(若不需相机数据，可删除对应代码，无需OpenCV环境）
- ROS2 HUMBLE
- pcl 1.12
- opencv

### 编译
``` bash
colcon build
```

### 转换
- `source install/setup.bash`
- config文件中修改`topic`-`lidar`为录制的`$lidar`
- 修改`bag_filepath`为采集的rosbag
- 修改`output_path`-`lidar`为输出pcd文件目录
- 执行 `ros2 launch bag_reading bag_reading_launch.py`
