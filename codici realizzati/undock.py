from flask import Flask, jsonify
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock  
import threading

app = Flask(__name__)

class UndockNode(Node):
    def __init__(self):
        super().__init__('undock_node')
        self.undock_client = ActionClient(self, Undock, '/undock')

    def send_undock_request(self):
        self.get_logger().info("Tentativo di inviare richiesta di undock...")

        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Il server dell'azione non è disponibile!")
            return False

        goal_msg = Undock.Goal()
        send_goal_future = self.undock_client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Il goal è stato rifiutato dal server dell'azione.")
                return False
            
            self.get_logger().info("Goal accettato, attendiamo il risultato...")
            result_future = goal_handle.get_result_async()

            def result_callback(future):
                result = future.result()
                if result:
                    self.get_logger().info("Undock completato con successo.")
                else:
                    self.get_logger().error("Errore nell'undock.")

            result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)
        return True

def start_ros2_node():
    rclpy.init()
    global undock_node
    undock_node = UndockNode()
    rclpy.spin(undock_node)
    undock_node.destroy_node()
    rclpy.shutdown()

undock_node = None
threading.Thread(target=start_ros2_node, daemon=True).start()

@app.route('/undock', methods=['POST'])
def undock():
    global undock_node
    if undock_node is None:
        return jsonify({"error": "Nodo ROS 2 non disponibile"}), 500

    success = undock_node.send_undock_request()
    if success:
        return jsonify({"status": "undocking avvenuto correttamente"}), 200
    else:
        return jsonify({"status": "undocking failed"}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
