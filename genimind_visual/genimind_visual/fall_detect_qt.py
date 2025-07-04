import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from cv_bridge import CvBridge
import sys


class FallDetectGUI(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"话题名称:{name}")
        self.subscriber = self.create_subscription(
            Image,
            "/genimind/fall_detect",
            self.image_callback,
            10
        )
        self.setWindowTitle("Fall Detection GUI")
        self.label_video = QLabel(self)
        self.main_widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.label_video)
        self.main_widget.setLayout(layout)
        self.setCentralWidget(self.main_widget)

    def convert_ros_to_qimage(self, msg):
        # 使用cv_bridge转换ROS消息为OpenCV格式
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # 注意BGR与RGB格式差异[1](@ref)
        # OpenCV转QImage
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        qimg = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return qimg.rgbSwapped()  # 转换BGR为RGB

    def image_callback(self, msg):
        # 处理接收到的图像消息
        self.get_logger().info("接收到图像消息")
        # 将ROS图像消息转换为QImage
        qimg = self.convert_ros_to_qimage(msg)
        # 显示图像
        self.label_video.setPixmap(QPixmap.fromImage(qimg))
        self.label_video.resize(qimg.size())
        self.label_video.show()
        self.label_video.setScaledContents(True)
        self.label_video.setGeometry(0, 0, 640, 480)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = FallDetectGUI("fall_detect_gui")
    rclpy.spin(node)
    rclpy.shutdown()
    sys.exit(app.exec_())
