from flask import Flask, jsonify
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import threading

app = Flask(__name__)

# Variabile per salvare lo stato della batteria
battery_status = {"voltage": None, "percentage": None, "current": None}

# Nodo ROS 2
class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.subscription = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)

    def battery_callback(self, msg):
        global battery_status
        battery_status = {
            "voltage": msg.voltage,
            "percentage": msg.percentage,
            "current": msg.current
        }

# Thread ROS 2
def start_ros2_node():
    rclpy.init()
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# Avvia il nodo ROS 2 in un thread separato
threading.Thread(target=start_ros2_node, daemon=True).start()

# Endpoint REST per ottenere lo stato della batteria
@app.route('/battery_status', methods=['GET'])
def get_battery_status():
    global battery_status
    if battery_status["voltage"] is None:
        return jsonify({"error": "Battery status not available yet"}), 503
    return jsonify(battery_status)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
