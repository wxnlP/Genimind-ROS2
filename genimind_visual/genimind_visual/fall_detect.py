import os
import rclpy
from rclpy.node import Node
from genimind_visual.utils.BpuDetect import BPU_Detect
from genimind_visual.utils.BpuDetect import CAM_DATA
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import serial
import time
import threading

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
        self.ser = None
        self.pic_save_path="src/genimind_visual/resource/result.jpg"
        self.cam_publisher = self.create_publisher(Image, "/genimind/fall_detect", 10)
        self.model_init()
        self.uart_init()
        # thread_uart = threading.Thread(target=self.uart_send_test)
        thread_detect = threading.Thread(target=self.detect)
        # thread_uart.start()
        # thread_detect.start()
        self.detect()
        # self.static_pic("/resource/1.jpg")

    def model_init(self):
        self.model = BPU_Detect(self.model_path, self.num_classes, coco_names, display=False, save_path=self.pic_save_path)

    def uart_init(self):
        try:
            self.ser = serial.Serial("/dev/ttyS1", int(9600), timeout=1) # 1s timeout
        except Exception as e:
            print("open serial failed!\n")

    def uart_send_test(self):
        while True:
            self.ser.write('F'.encode("utf-8"))
            self.get_logger().info("发送跌倒预警")
            time.sleep(0.5)


    def uart_send_fall(self):
        self.ser.write('F'.encode("utf-8"))
        self.get_logger().info("发送跌倒预警")

    def detect(self):
        data = CAM_DATA()
        self.model.camera_init()
        flag = 0
        fall_frames = []
        while True:
            data = self.model.camera_detect()
            flag += 1
            print(data.data_label)
            if coco_names[1] in data.data_label:
                fall_frames.append(1)
                self.get_logger().info("跌倒变量+1")
            if flag >= 10:
                if len(fall_frames) >= 5:
                    self.get_logger().info("检测到跌倒")
                    self.uart_send_fall()
                    fall_frames = []
                    flag=0
                else:
                    self.get_logger().info("未检测到跌倒")
                    fall_frames = []  
                    flag=0      
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
    node = FallDetect("detect_node", "/models/genimind_fall_1.bin", 2)  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

