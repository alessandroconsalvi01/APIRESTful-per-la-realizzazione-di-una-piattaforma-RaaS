from flask import Flask, jsonify
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from irobot_create_msgs.msg import DockStatus
import threading

app = Flask(__name__)
dock_status = {"docked": False, "charging": False}

class DockStatusNode(Node):
    def __init__(self):
        super().__init__('dock_status_node')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(DockStatus,'/dock_status',self.dock_callback,qos_profile)
        self.last_status = None  
        self.get_logger().info("Sottoscrizione al topic /dock_status avviata correttamente")

    def dock_callback(self, msg):
        global dock_status
        new_status = {
            "docked": msg.is_docked,
            "charging": False
        }

        if new_status != self.last_status:
            self.get_logger().info(f"Dock status aggiornato: {new_status}")
            self.last_status = new_status

        dock_status = new_status

def start_ros2_node():
    rclpy.init()
    node = DockStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

threading.Thread(target=start_ros2_node, daemon=True).start()

@app.route('/dock_status', methods=['GET'])
def get_dock_status():
    global dock_status
    return jsonify(dock_status)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
