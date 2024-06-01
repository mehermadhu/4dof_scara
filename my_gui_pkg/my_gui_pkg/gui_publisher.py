import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton

class GUIPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.init_ui()

    def init_ui(self):
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('Control Command Publisher')

        layout = QVBoxLayout()

        self.entries = []
        for i in range(4):
            box = QHBoxLayout()
            label = QLabel(f"Value {i+1}:")
            entry = QLineEdit()
            box.addWidget(label)
            box.addWidget(entry)
            layout.addLayout(box)
            self.entries.append(entry)

        publish_button = QPushButton('Publish')
        publish_button.clicked.connect(self.publish_values)
        layout.addWidget(publish_button)

        self.window.setLayout(layout)
        self.window.show()
        sys.exit(self.app.exec_())

    def publish_values(self):
        msg = Float64MultiArray()
        try:
            msg.data = [float(entry.text()) for entry in self.entries[:2]]
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
        except ValueError:
            self.get_logger().error('All inputs must be floating-point numbers.')

def main(args=None):
    rclpy.init(args=args)
    gui_publisher = GUIPublisher()
    rclpy.spin(gui_publisher)
    gui_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
