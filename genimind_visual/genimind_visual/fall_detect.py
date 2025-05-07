import os
import rclpy
from rclpy.node import Node
from genimind_visual.utils.BpuDetect import BPU_Detect
from genimind_visual.utils.BpuDetect import CAM_DATA
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

coco_names = [
    "person",
    "fall"
    ]

class FallDetect(Node):
    def __init__(self, node, models_path, num_classes):
        super().__init__(node)
        self.node = node
        # 格式转换对象
        self.bridge = CvBridge()
        # 拼接models路径
        self.model_path = get_package_share_directory("genimind_visual") + models_path 
        self.get_logger().info(f"节点名称为{self.node}")
        self.num_classes = num_classes
        self.model = None
        self.pic_save_path="src/genimind_visual/resource/result.jpg"
        self.cam_publisher = self.create_publisher(Image, "/genimind/fall_detect", 10)
        self.model_init()
        self.detect()
        # self.static_pic("/resource/1.jpg")

    def model_init(self):
        self.model = BPU_Detect(self.model_path, self.num_classes, coco_names, display=False, save_path=self.pic_save_path)

    def detect(self):
        data = CAM_DATA()
        self.model.camera_init()
        while True:
            data = self.model.camera_detect()
            data.print_properties()
            ros_img = self.bridge.cv2_to_imgmsg(data.data_img_cv2)
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = "camera"
            self.cam_publisher.publish(ros_img)
            self.get_logger().info('发布图像帧')

    def static_pic(self, pic_path):
        img = get_package_share_directory("genimind_visual") + pic_path
        print(img)
        self.model.static_image_detect(img)


def main(args=None):
    """
    ros2运行该节点的入口函数
    """
    rclpy.init(args=args) # 初始化rclpy
    node = FallDetect("detect_node", "/models/genimind_fall.bin", 3)  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

