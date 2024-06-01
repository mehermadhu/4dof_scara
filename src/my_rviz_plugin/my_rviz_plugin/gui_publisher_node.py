import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton

class GuiPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_values', 10)
        self.init_ui()

    def init_ui(self):
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('Joint Values Publisher')

        self.layout = QVBoxLayout()

        self.joint1_layout = QHBoxLayout()
        self.joint1_label = QLabel("Joint 1 Value:")
        self.joint1_entry = QLineEdit()
        self.joint1_layout.addWidget(self.joint1_label)
        self.joint1_layout.addWidget(self.joint1_entry)
        self.layout.addLayout(self.joint1_layout)

        self.joint2_layout = QHBoxLayout()
        self.joint2_label = QLabel("Joint 2 Value:")
        self.joint2_entry = QLineEdit()
        self.joint2_layout.addWidget(self.joint2_label)
        self.joint2_layout.addWidget(self.joint2_entry)
        self.layout.addLayout(self.joint2_layout)

        self.publish_button = QPushButton("Publish")
        self.publish_button.clicked.connect(self.publish_values)
        self.layout.addWidget(self.publish_button)

        self.window.setLayout(self.layout)
        self.window.show()
        self.app.exec_()

    def publish_values(self):
        try:
            joint1_value = float(self.joint1_entry.text())
            joint2_value = float(self.joint2_entry.text())
        except ValueError:
            self.get_logger().error('Invalid input, please enter numeric values.')
            return

        msg = Float64MultiArray()
        msg.data = [joint1_value, joint2_value]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: [{joint1_value}, {joint2_value}]')

def main(args=None):
    rclpy.init(args=args)
    gui_publisher = GuiPublisher()
    rclpy.spin(gui_publisher)
    gui_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
