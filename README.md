```
genimind_base ---------- 底盘通信功能包
genimind_interfaces ---- 底盘消息接口功能包
```

``` shell
source install/setup.bash
# 启动底盘 
ros2 launch genimind_base genimind_base.launch.py open_can:=True
# 验证can通信
ros2 topic echo /genimind_status genimind_interfaces/msg/GenimindStatus
# 启动雷达
ros2 launch sllidar_ros2 sllidar_a1_launch.py
# 加载机器人模型
ros2 launch genimind_description genimind_model.launch.py
# 启动slam_toolbox建图
ros2 launch genimind_slam_toolbox online_async.launch.py
# 启动rqt验证TF树
rqt
# 启动rviz2查看地图
rviz2
# 启动通用键盘控制节点
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

## 环境配置

``` shell

# cartographer建图
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
# nav2导航框架
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

```