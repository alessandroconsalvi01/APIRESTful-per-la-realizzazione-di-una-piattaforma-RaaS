import math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist
from flask import Flask, jsonify, request
import threading

rclpy.init()
app = Flask(__name__)

class TurtlebotControlNode(Node):
    def __init__(self):
        super().__init__('turtlebot_control_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing velocity: linear_x={linear_x}, angular_z={angular_z}')

turtlebot_control_node = TurtlebotControlNode()

@app.route('/move', methods=['POST'])
def move():
    try:
        if not request.is_json:
            return jsonify({'status': 'error', 'message': 'La richiesta non è in formato JSON'}), 400
        
        data = request.get_json()

        if 'linear_x' not in data or 'angular_z' not in data:
            return jsonify({'status': 'error', 'message': "'linear_x' o 'angular_z' non trovato nei dati"}), 400

        linear_x = float(data['linear_x'])
        angular_z = float(data['angular_z'])
        turtlebot_control_node.send_velocity(linear_x, angular_z)

        return jsonify({'status': 'success', 'message': f'Movimento avviato con linear_x={linear_x} e angular_z={angular_z}'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/stop', methods=['POST'])
def stop():
    try:
        turtlebot_control_node.send_velocity(0.0, 0.0)
        return jsonify({'status': 'success', 'message': 'Movimento fermato'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

if __name__ == '__main__':
    def flask_thread():
        app.run(host='0.0.0.0', port=5000)
    threading.Thread(target=flask_thread).start()
    rclpy.spin(turtlebot_control_node)
