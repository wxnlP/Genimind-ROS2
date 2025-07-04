import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget

class RosImageSubscriber(QObject):
    image_signal = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.node = Node("pyqt_image_subscriber")
        self.subscription = self.node.create_subscription(
            Image,
            '/genimind/fall_detect',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(
                cv_image.data, 
                w, h, 
                bytes_per_line, 
                QImage.Format_BGR888
            )
            self.image_signal.emit(qt_image)
        except Exception as e:
            self.node.get_logger().error(f"Image processing error: {str(e)}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_ros()

    def init_ui(self):
        self.setWindowTitle("ROS2 Image Viewer")
        self.setGeometry(100, 100, 800, 600)
        
        central_widget = QWidget()
        self.layout = QVBoxLayout(central_widget)
        
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        
        self.layout.addWidget(self.image_label)
        self.setCentralWidget(central_widget)

    def init_ros(self):
        self.ros_thread = QThread()
        self.ros_subscriber = RosImageSubscriber()
        self.ros_subscriber.moveToThread(self.ros_thread)
        
        self.ros_thread.started.connect(
            lambda: self.ros_subscriber.node.get_logger().info("ROS thread started")
        )
        self.ros_subscriber.image_signal.connect(self.update_image)
        
        self.ros_thread.start()

    def update_image(self, qt_image):
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.image_label.setPixmap(scaled_pixmap)

    def closeEvent(self, event):
        self.ros_subscriber.node.destroy_node()
        self.ros_thread.quit()
        self.ros_thread.wait()
        super().closeEvent(event)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())