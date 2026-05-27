import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


class VDA5050AdapterNode(Node):
    """Minimal VDA5050 MQTT adapter for navigation orders and state."""

    def __init__(self):
        super().__init__('vda5050_adapter_node')

        self.declare_parameter('broker_host', 'localhost')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('interface_name', 'vda5050')
        self.declare_parameter('major_version', 'v3')
        self.declare_parameter('manufacturer', 'helper')
        self.declare_parameter('serial_number', 'helper_001')
        self.declare_parameter('goal_topic', '/planning/goal_pose')
        self.declare_parameter('behavior_cmd_topic', '/planning/behavior_cmd')
        self.declare_parameter('behavior_state_topic', '/planning/behavior_state')
        self.declare_parameter('odom_topic', '/control/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('state_publish_rate_hz', 2.0)

        self.interface_name = self.get_parameter('interface_name').value
        self.major_version = self.get_parameter('major_version').value
        self.manufacturer = self.get_parameter('manufacturer').value
        self.serial_number = self.get_parameter('serial_number').value
        self.map_frame = self.get_parameter('map_frame').value

        self.order_id = ''
        self.order_update_id = 0
        self.behavior_state = 'run'
        self.last_odom = None
        self.mqtt_client = None
        self.mqtt_connected = False
        self.mqtt_lock = threading.Lock()

        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('goal_topic').value,
            10,
        )
        self.behavior_cmd_pub = self.create_publisher(
            String,
            self.get_parameter('behavior_cmd_topic').value,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('behavior_state_topic').value,
            self.behavior_state_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )

        self.connect_mqtt()
        rate = float(self.get_parameter('state_publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 0.1), self.publish_state)

    def connect_mqtt(self):
        if mqtt is None:
            self.get_logger().error(
                'paho-mqtt is not installed. Install python3-paho-mqtt.'
            )
            return

        client_id = f'{self.manufacturer}_{self.serial_number}_adapter'
        self.mqtt_client = mqtt.Client(client_id=client_id)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message

        host = self.get_parameter('broker_host').value
        port = int(self.get_parameter('broker_port').value)
        try:
            self.mqtt_client.connect(host, port, keepalive=30)
        except Exception as exc:
            self.get_logger().error(f'MQTT connect failed: {exc}')
            return

        self.mqtt_client.loop_start()
        self.get_logger().info(f'connecting MQTT broker {host}:{port}')

    def on_mqtt_connect(self, client, _userdata, _flags, rc):
        if rc != 0:
            self.get_logger().error(f'MQTT connection rejected rc={rc}')
            return

        self.mqtt_connected = True
        client.subscribe(self.topic('order'), qos=0)
        client.subscribe(self.topic('instantActions'), qos=0)
        self.get_logger().info(
            f'MQTT connected; subscribed to {self.topic("order")} and '
            f'{self.topic("instantActions")}'
        )

    def on_mqtt_disconnect(self, _client, _userdata, rc):
        self.mqtt_connected = False
        self.get_logger().warn(f'MQTT disconnected rc={rc}')

    def on_mqtt_message(self, _client, _userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
        except Exception as exc:
            self.get_logger().warn(f'invalid MQTT JSON on {msg.topic}: {exc}')
            return

        if msg.topic.endswith('/order'):
            self.handle_order(payload)
        elif msg.topic.endswith('/instantActions'):
            self.handle_instant_actions(payload)

    def handle_order(self, order):
        node = self.extract_target_node(order)
        if node is None:
            self.get_logger().warn('order ignored: no nodePosition found')
            return

        position = node.get('nodePosition', {})
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = position.get('mapId') or self.map_frame
        goal.pose.position.x = float(position.get('x', 0.0))
        goal.pose.position.y = float(position.get('y', 0.0))
        theta = float(position.get('theta', 0.0))
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)

        self.order_id = str(order.get('orderId', ''))
        self.order_update_id = int(order.get('orderUpdateId', 0))
        self.goal_pub.publish(goal)
        self.get_logger().info(
            'published VDA5050 goal '
            f'order_id={self.order_id} '
            f'x={goal.pose.position.x:.3f} '
            f'y={goal.pose.position.y:.3f} '
            f'theta={theta:.3f}'
        )

    def extract_target_node(self, order):
        nodes = order.get('nodes', [])
        released_nodes = [
            node for node in nodes
            if node.get('released', False) and 'nodePosition' in node
        ]
        if released_nodes:
            return released_nodes[-1]

        positioned_nodes = [
            node for node in nodes
            if 'nodePosition' in node
        ]
        if positioned_nodes:
            return positioned_nodes[-1]
        return None

    def handle_instant_actions(self, message):
        actions = (
            message.get('instantActions')
            or message.get('actions')
            or []
        )
        for action in actions:
            action_type = str(action.get('actionType', '')).lower()
            if action_type in {'stop', 'pause', 'cancelorder'}:
                self.publish_behavior_cmd('stop')
            elif action_type in {'start', 'resume'}:
                self.publish_behavior_cmd('run')
            else:
                self.get_logger().info(
                    f'unsupported instant action ignored: {action_type}'
                )

    def publish_behavior_cmd(self, behavior):
        msg = String()
        msg.data = behavior
        self.behavior_cmd_pub.publish(msg)
        self.get_logger().info(f'published behavior command from VDA5050: {behavior}')

    def behavior_state_callback(self, msg):
        self.behavior_state = msg.data.strip().lower()

    def odom_callback(self, msg):
        self.last_odom = msg

    def publish_state(self):
        if self.mqtt_client is None or not self.mqtt_connected:
            return

        state = self.build_state_message()
        with self.mqtt_lock:
            self.mqtt_client.publish(
                self.topic('state'),
                json.dumps(state),
                qos=0,
            )

    def build_state_message(self):
        pose = self.current_pose()
        return {
            'headerId': int(time.time() * 1000) % 2147483647,
            'timestamp': self.get_clock().now().to_msg().sec,
            'version': '3.0.0',
            'manufacturer': self.manufacturer,
            'serialNumber': self.serial_number,
            'orderId': self.order_id,
            'orderUpdateId': self.order_update_id,
            'lastNodeId': '',
            'lastNodeSequenceId': 0,
            'nodeStates': [],
            'edgeStates': [],
            'actionStates': [],
            'batteryState': {
                'batteryCharge': 0.0,
                'charging': False,
            },
            'operatingMode': 'AUTOMATIC',
            'errors': [],
            'information': [{
                'infoType': 'behavior_state',
                'infoLevel': 'INFO',
                'infoDescription': self.behavior_state,
            }],
            'safetyState': {
                'eStop': 'NONE',
                'fieldViolation': self.behavior_state == 'stop',
            },
            'agvPosition': pose,
            'driving': self.behavior_state not in {'stop'},
        }

    def current_pose(self):
        if self.last_odom is None:
            return {
                'positionInitialized': False,
                'mapId': self.map_frame,
                'x': 0.0,
                'y': 0.0,
                'theta': 0.0,
            }

        pose = self.last_odom.pose.pose
        return {
            'positionInitialized': True,
            'mapId': self.map_frame,
            'x': pose.position.x,
            'y': pose.position.y,
            'theta': self.yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        }

    def topic(self, name):
        return (
            f'{self.interface_name}/{self.major_version}/'
            f'{self.manufacturer}/{self.serial_number}/{name}'
        )

    @staticmethod
    def yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self):
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VDA5050AdapterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
