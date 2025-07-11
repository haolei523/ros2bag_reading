### ros2 bag 转换 pcd
# 采集激光雷达数据
``` bash
lidar = /lidar
ros2 bag record $lidar
```

# 编译
``` bash
colcon build
```

# 转换
- `source install/setup.bash`
- config文件中修改topic-lidar为录制$lidar
- 修改output_path-lidar为输出pcd文件目录
