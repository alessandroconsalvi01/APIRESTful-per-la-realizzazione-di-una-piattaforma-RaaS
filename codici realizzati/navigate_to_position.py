import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import NavigateToPosition
from flask import Flask, jsonify, request

rclpy.init()
app = Flask(__name__)

action_status = None

class TurtlebotNavigateClient(Node):
    def __init__(self):
        super().__init__('turtlebot_navigate_client')
        self._navigate_client = ActionClient(self, NavigateToPosition, 'navigate_to_position')

    def send_navigation_goal(self, target_pose):
        goal_msg = NavigateToPosition.Goal()
        goal_msg.goal_pose = target_pose
        send_goal_future = self._navigate_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        global action_status
        result = future.result()
        status = result.status
        if status == 3: 
            action_status = 'success'
            self.get_logger().info("Navigazione completata con successo.")
        else:
            action_status = 'aborted'
            self.get_logger().error(f"Errore nella navigazione, stato: {status}")

turtlebot_navigate_client = TurtlebotNavigateClient()

@app.route('/navigate', methods=['POST'])
def navigate():
    global action_status
    try:
        if not request.is_json:
            return jsonify({'status': 'error', 'message': 'La richiesta non è in formato JSON'}), 400
        
        data = request.get_json()

        if 'x' not in data or 'y' not in data or 'angle' not in data:
            return jsonify({'status': 'error', 'message': "'x', 'y' o 'angle' non trovato nei dati"}), 400
        
        x = float(data['x'])
        y = float(data['y'])
        angle_deg = float(data['angle'])
        angle_rad = math.radians(angle_deg)

        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.orientation.z = math.sin(angle_rad / 2)
        target_pose.pose.orientation.w = math.cos(angle_rad / 2)

        turtlebot_navigate_client.send_navigation_goal(target_pose)
        rclpy.spin_once(turtlebot_navigate_client, timeout_sec=10)
        
        if action_status == 'success':
            return jsonify({'status': 'success', 'message': f'Navigazione avviata verso ({x}, {y}) con orientamento {angle_deg} gradi'}), 200
        elif action_status == 'aborted':
            return jsonify({'status': 'error', 'message': 'Navigazione fallita, l\'azione è stata abortita.'}), 500
        else:
            return jsonify({'status': 'error', 'message': 'Lo stato dell\'azione non è ancora disponibile.'}), 503

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

if __name__ == '__main__':
    import threading
    def flask_thread():
        app.run(host='0.0.0.0', port=5000)
    threading.Thread(target=flask_thread).start()
    rclpy.spin(turtlebot_navigate_client)
