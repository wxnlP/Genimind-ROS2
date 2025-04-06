import os
import rclpy
from rclpy.node import Node
from genimind_visual.utils.BpuDetect import BPU_Detect
from ament_index_python.packages import get_package_share_directory


class FallDetect(Node):
    def __init__(self, node, models_path):
        super().__init__(node)
        self.node = node
        # 拼接models路径
        self.model_path = get_package_share_directory("genimind_visual") + models_path 
        self.get_logger().info(f"节点名称为{self.node}")
        self.detect()

    def detect(self):
        detect = BPU_Detect(self.model_path, display=False)
        detect.camera_init()
        detect.camera_detect()


def main(args=None):
    """
    ros2运行该节点的入口函数
    """
    rclpy.init(args=args) # 初始化rclpy
    node = FallDetect("detect_node", "/models/yolo11n_detect_bayese_640x640_nv12.bin")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

