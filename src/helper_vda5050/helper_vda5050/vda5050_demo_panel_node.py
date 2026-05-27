import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer

import rclpy
from rclpy.node import Node

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


class VDA5050DemoPanelNode(Node):
    """Small local web panel for publishing demo VDA5050 messages."""

    def __init__(self):
        super().__init__('vda5050_demo_panel_node')

        self.declare_parameter('broker_host', 'localhost')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('interface_name', 'vda5050')
        self.declare_parameter('major_version', 'v3')
        self.declare_parameter('manufacturer', 'helper')
        self.declare_parameter('serial_number', 'helper_001')
        self.declare_parameter('http_host', '127.0.0.1')
        self.declare_parameter('http_port', 8088)
        self.declare_parameter('default_map_id', 'odom')

        self.interface_name = self.get_parameter('interface_name').value
        self.major_version = self.get_parameter('major_version').value
        self.manufacturer = self.get_parameter('manufacturer').value
        self.serial_number = self.get_parameter('serial_number').value
        self.default_map_id = self.get_parameter('default_map_id').value

        self.mqtt_client = None
        self.mqtt_connected = False
        self.latest_state = {}
        self.last_publish = {}
        self.lock = threading.Lock()

        self.connect_mqtt()
        self.start_http_server()

    def connect_mqtt(self):
        if mqtt is None:
            self.get_logger().error(
                'paho-mqtt is not installed. Install python3-paho-mqtt.'
            )
            return

        client_id = f'{self.manufacturer}_{self.serial_number}_demo_panel'
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

    def on_mqtt_connect(self, client, _userdata, _flags, rc):
        if rc != 0:
            self.get_logger().error(f'MQTT connection rejected rc={rc}')
            return

        self.mqtt_connected = True
        client.subscribe(self.topic('state'), qos=0)
        self.get_logger().info(f'MQTT connected; subscribed to {self.topic("state")}')

    def on_mqtt_disconnect(self, _client, _userdata, rc):
        self.mqtt_connected = False
        self.get_logger().warn(f'MQTT disconnected rc={rc}')

    def on_mqtt_message(self, _client, _userdata, msg):
        if not msg.topic.endswith('/state'):
            return

        try:
            state = json.loads(msg.payload.decode('utf-8'))
        except Exception as exc:
            self.get_logger().warn(f'invalid state JSON: {exc}')
            return

        with self.lock:
            self.latest_state = state

    def start_http_server(self):
        host = self.get_parameter('http_host').value
        port = int(self.get_parameter('http_port').value)
        handler = self.make_handler()
        self.http_server = ThreadingHTTPServer((host, port), handler)
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(f'VDA5050 demo panel: http://{host}:{port}')

    def make_handler(self):
        node = self

        class DemoPanelHandler(BaseHTTPRequestHandler):
            def log_message(self, _format, *args):
                return

            def do_GET(self):
                if self.path == '/':
                    self.send_html(node.html_page())
                elif self.path == '/api/state':
                    self.send_json(node.status_payload())
                else:
                    self.send_error(404)

            def do_POST(self):
                length = int(self.headers.get('Content-Length', '0'))
                raw_body = self.rfile.read(length).decode('utf-8')
                try:
                    body = json.loads(raw_body) if raw_body else {}
                except json.JSONDecodeError:
                    self.send_error(400, 'invalid JSON')
                    return

                if self.path == '/api/order':
                    response = node.publish_order(body)
                    self.send_json(response)
                elif self.path == '/api/instant_action':
                    response = node.publish_instant_action(body)
                    self.send_json(response)
                else:
                    self.send_error(404)

            def send_html(self, html):
                data = html.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def send_json(self, payload):
                data = json.dumps(payload).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        return DemoPanelHandler

    def publish_order(self, body):
        x = float(body.get('x', 1.0))
        y = float(body.get('y', 0.0))
        theta = float(body.get('theta', 0.0))
        map_id = str(body.get('mapId') or self.default_map_id)
        order_id = str(body.get('orderId') or f'demo_order_{int(time.time())}')

        order = {
            'orderId': order_id,
            'orderUpdateId': 0,
            'nodes': [
                {
                    'nodeId': 'start',
                    'sequenceId': 0,
                    'released': True,
                    'nodePosition': {
                        'x': 0.0,
                        'y': 0.0,
                        'theta': 0.0,
                        'mapId': map_id,
                    },
                },
                {
                    'nodeId': 'goal',
                    'sequenceId': 2,
                    'released': True,
                    'nodePosition': {
                        'x': x,
                        'y': y,
                        'theta': theta,
                        'mapId': map_id,
                    },
                },
            ],
            'edges': [
                {
                    'edgeId': 'edge_1',
                    'sequenceId': 1,
                    'startNodeId': 'start',
                    'endNodeId': 'goal',
                    'released': True,
                },
            ],
        }
        return self.publish_mqtt('order', order)

    def publish_instant_action(self, body):
        action_type = str(body.get('actionType', 'stop'))
        message = {
            'headerId': int(time.time() * 1000) % 2147483647,
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'version': '3.0.0',
            'manufacturer': self.manufacturer,
            'serialNumber': self.serial_number,
            'instantActions': [
                {
                    'actionId': f'{action_type}_{int(time.time())}',
                    'actionType': action_type,
                    'blockingType': body.get('blockingType', 'NONE'),
                },
            ],
        }
        return self.publish_mqtt('instantActions', message)

    def publish_mqtt(self, name, payload):
        if self.mqtt_client is None or not self.mqtt_connected:
            return {
                'ok': False,
                'error': 'MQTT is not connected',
                'topic': self.topic(name),
            }

        topic = self.topic(name)
        self.mqtt_client.publish(topic, json.dumps(payload), qos=0)
        with self.lock:
            self.last_publish = {
                'topic': topic,
                'payload': payload,
                'time': time.time(),
            }
        self.get_logger().info(f'published demo {name} to {topic}')
        return {'ok': True, 'topic': topic, 'payload': payload}

    def status_payload(self):
        with self.lock:
            return {
                'mqtt_connected': self.mqtt_connected,
                'topics': {
                    'order': self.topic('order'),
                    'instantActions': self.topic('instantActions'),
                    'state': self.topic('state'),
                },
                'latest_state': self.latest_state,
                'last_publish': self.last_publish,
            }

    def topic(self, name):
        return (
            f'{self.interface_name}/{self.major_version}/'
            f'{self.manufacturer}/{self.serial_number}/{name}'
        )

    def html_page(self):
        return HTML_PAGE

    def destroy_node(self):
        if hasattr(self, 'http_server'):
            self.http_server.shutdown()
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>VDA5050 Demo Panel</title>
  <style>
    body {
      background: #f6f8fb;
      color: #1f2933;
      font-family: Arial, sans-serif;
      margin: 0;
    }
    main {
      margin: 0 auto;
      max-width: 980px;
      padding: 28px;
    }
    h1 {
      font-size: 26px;
      margin: 0 0 18px;
    }
    section {
      background: #ffffff;
      border: 1px solid #d8dee9;
      border-radius: 8px;
      margin: 0 0 16px;
      padding: 18px;
    }
    .grid {
      display: grid;
      gap: 12px;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
    }
    label {
      display: block;
      font-size: 13px;
      font-weight: 700;
      margin-bottom: 4px;
    }
    input {
      border: 1px solid #aeb7c2;
      border-radius: 6px;
      box-sizing: border-box;
      font-size: 15px;
      padding: 9px;
      width: 100%;
    }
    button {
      background: #0052cc;
      border: 0;
      border-radius: 6px;
      color: #ffffff;
      cursor: pointer;
      font-size: 15px;
      font-weight: 700;
      margin: 8px 8px 0 0;
      padding: 10px 14px;
    }
    button.stop {
      background: #bf2600;
    }
    button.resume {
      background: #00875a;
    }
    pre {
      background: #172b4d;
      border-radius: 8px;
      color: #e6fcff;
      min-height: 120px;
      overflow: auto;
      padding: 14px;
      white-space: pre-wrap;
    }
    .status {
      color: #42526e;
      font-size: 14px;
      margin-top: 8px;
    }
  </style>
</head>
<body>
  <main>
    <h1>VDA5050 Demo Panel</h1>

    <section>
      <h2>Order</h2>
      <div class="grid">
        <div>
          <label for="x">x</label>
          <input id="x" type="number" step="0.1" value="1.0">
        </div>
        <div>
          <label for="y">y</label>
          <input id="y" type="number" step="0.1" value="0.0">
        </div>
        <div>
          <label for="theta">theta rad</label>
          <input id="theta" type="number" step="0.1" value="0.0">
        </div>
        <div>
          <label for="mapId">mapId</label>
          <input id="mapId" value="odom">
        </div>
      </div>
      <button onclick="sendOrder()">Send Order</button>
      <div class="status" id="orderStatus"></div>
    </section>

    <section>
      <h2>Instant Actions</h2>
      <button class="stop" onclick="sendAction('stop', 'HARD')">Stop</button>
      <button class="resume" onclick="sendAction('resume', 'NONE')">Resume</button>
      <button onclick="sendAction('pause', 'HARD')">Pause</button>
      <button onclick="sendAction('cancelOrder', 'HARD')">Cancel Order</button>
      <div class="status" id="actionStatus"></div>
    </section>

    <section>
      <h2>MQTT / State</h2>
      <pre id="state">Loading...</pre>
    </section>
  </main>

  <script>
    async function postJson(url, payload) {
      const response = await fetch(url, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload),
      });
      return response.json();
    }

    async function sendOrder() {
      const payload = {
        x: Number(document.getElementById('x').value),
        y: Number(document.getElementById('y').value),
        theta: Number(document.getElementById('theta').value),
        mapId: document.getElementById('mapId').value,
      };
      const result = await postJson('/api/order', payload);
      document.getElementById('orderStatus').textContent =
        result.ok ? `Published to ${result.topic}` : `Error: ${result.error}`;
      refreshState();
    }

    async function sendAction(actionType, blockingType) {
      const result = await postJson('/api/instant_action', {
        actionType,
        blockingType,
      });
      document.getElementById('actionStatus').textContent =
        result.ok ? `Published ${actionType} to ${result.topic}` : `Error: ${result.error}`;
      refreshState();
    }

    async function refreshState() {
      const response = await fetch('/api/state');
      const payload = await response.json();
      document.getElementById('state').textContent =
        JSON.stringify(payload, null, 2);
    }

    refreshState();
    setInterval(refreshState, 1000);
  </script>
</body>
</html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = VDA5050DemoPanelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
