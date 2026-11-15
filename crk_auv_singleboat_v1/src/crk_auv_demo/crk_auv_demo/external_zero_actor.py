"""Exercise the real external observation/action round trip using a zero residual.

Replace inference only after restoring the correct role, model and any context.
This diagnostic never loads a historical checkpoint or pretends to be trained RL.
"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from crk_auv_demo.policy import vector3


class ExternalZeroActor(Node):
    def __init__(self):
        super().__init__('crk_external_zero_actor')
        self.publisher = self.create_publisher(String, '/auv_0/policy/action', 10)
        self.subscription = self.create_subscription(
            String, '/auv_0/policy/observation', self.respond, 10)
        self.get_logger().info('Diagnostic external zero residual; no trained model loaded.')

    def respond(self, message):
        packet = json.loads(message.data)
        if packet['schema'] != 'crk.observation28.v1' or len(packet['observation']) != 28:
            raise ValueError('Unsupported observation request')
        action = vector3([0., 0., 0.])
        response = {'schema': 'crk.residual3.v1', 'seq': packet['seq'],
                    'policy_contract_sha256': packet['policy_contract_sha256'],
                    'observation_sim_s': packet['sim_s'], 'action': action.tolist()}
        self.publisher.publish(String(data=json.dumps(response, allow_nan=False)))


def main():
    rclpy.init()
    node = ExternalZeroActor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
