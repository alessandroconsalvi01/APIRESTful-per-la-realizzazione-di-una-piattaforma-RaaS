import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from flask import Flask, request, jsonify
import math

app = Flask(__name__)

class RotateTurtleBot(Node):
    def __init__(self):
        super().__init__('rotate_turtlebot')
        self._action_client = ActionClient(self, RotateAngle, 'rotate_angle')

    def send_rotate_goal(self, angle):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        self.get_logger().info(f'Inviando il goal di rotazione: {angle} radianti')
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

rotate_turtlebot = None

@app.route('/rotate', methods=['POST'])
def rotate():
    try:
        if not request.is_json:
            return jsonify({'status': 'error', 'message': 'La richiesta non è in formato JSON'}), 400

        data = request.get_json()

        if 'angle' not in data:
            return jsonify({'status': 'error', 'message': "'angle' non trovato nei dati"}), 400
        if 'direction' not in data:
            return jsonify({'status': 'error', 'message': "'direction' non trovato nei dati"}), 400

        angle_deg = float(data['angle'])
        direction = data['direction'].lower()
        angle_rad = math.radians(angle_deg)

        if direction == 'oraria':
            angle_rad = -abs(angle_rad)
        elif direction == 'antioraria':
            angle_rad = abs(angle_rad)
        else:
            return jsonify({'status': 'error', 'message': "Direzione non valida. Usa 'oraria' o 'antioraria'."}), 400

        future = rotate_turtlebot.send_rotate_goal(angle_rad)
        future.add_done_callback(lambda future: print(f'Rotazione completata di {angle_rad} radianti'))
        
        return jsonify({'status': 'success', 'message': f'Rotazione avviata di {angle_deg} gradi ({angle_rad} radian) in direzione {direction}'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400


if __name__ == '__main__':
    rclpy.init()
    rotate_turtlebot = RotateTurtleBot()

    from threading import Thread
    def run_flask():
        app.run(host='0.0.0.0', port=5000)
    
    flask_thread = Thread(target=run_flask)
    flask_thread.start()
    
    try:
        rclpy.spin(rotate_turtlebot)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
