import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton
import rviz_common

class JointValuesPanel(rviz_common.Panel):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('JointValuesPanel')

        self.node = rclpy.create_node('joint_values_panel')

        self.publisher_ = self.node.create_publisher(Float64MultiArray, 'joint_values', 10)

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

        self.setLayout(self.layout)

    def publish_values(self):
        joint1_value = float(self.joint1_entry.text())
        joint2_value = float(self.joint2_entry.text())
        msg = Float64MultiArray()
        msg.data = [joint1_value, joint2_value]
        self.publisher_.publish(msg)
        self.node.get_logger().info(f"Published: [{joint1_value}, {joint2_value}]")

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    main_window = JointValuesPanel()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
