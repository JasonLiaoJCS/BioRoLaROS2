"""ROS subscriptions and a read-only, dependency-free HTTP interface."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from http.cookies import SimpleCookie, CookieError
from importlib.resources import files
import json
import math
import os
from pathlib import Path
import secrets
import threading
from urllib.parse import urlsplit, parse_qs

from .model import MonitorStore, TOPICS


def make_handler(store, token):
    page = files('rinbo_monitor').joinpath('panel.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def authenticated(self):
            try:
                cookie = SimpleCookie(self.headers.get('Cookie', '')).get('rinbo_monitor_token')
                return cookie is not None and secrets.compare_digest(cookie.value.encode('utf-8'), token.encode('utf-8'))
            except (CookieError, TypeError):
                return False

        def send_json(self, payload, status=200):
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if not self.authenticated():
                self.send_json({'error': '請使用 terminal 的存取連結重新登入'}, 401)
                return
            # This custom header prevents cross-origin forms from controlling
            # recordings. No CORS support or robot-control route is provided.
            if self.headers.get('X-Rinbo-Recording') != '1':
                self.send_json({'error': 'Missing recording request header'}, 403)
                return
            actions = {'/api/recording/start': store.start_recording,
                       '/api/recording/stop': store.stop_recording}
            action = actions.get(self.path)
            if action is None:
                self.send_json({'error': 'Unknown recording action'}, 404)
                return
            try:
                self.send_json(action())
            except ValueError as error:
                self.send_json({'error': str(error)}, 409)
            except OSError as error:
                self.send_json({'error': '無法建立錄製檔案：' + str(error)}, 500)

        def do_GET(self):
            url = urlsplit(self.path)
            path = url.path
            supplied = parse_qs(url.query).get('token', [''])[0]
            if path == '/' and secrets.compare_digest(supplied.encode('utf-8'), token.encode('utf-8')):
                # Remove the token from the address bar after establishing a
                # host-only session. Never log the request URL or credentials.
                self.send_response(303)
                self.send_header('Location', '/')
                self.send_header('Set-Cookie', 'rinbo_monitor_token={}; HttpOnly; SameSite=Strict; Path=/'.format(token))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Referrer-Policy', 'no-referrer')
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            if not self.authenticated():
                self.send_error(401, 'Open the access link printed in the Orin terminal.')
                return
            if path == '/api/recording/download':
                try:
                    recording_path = store.recording_path()
                    with recording_path.open('rb') as recording_file:
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json; charset=utf-8')
                        self.send_header('Content-Length', str(os.fstat(recording_file.fileno()).st_size))
                        self.send_header('Content-Disposition', 'attachment; filename="{}"'.format(recording_path.name))
                        self.send_header('Cache-Control', 'no-store')
                        self.end_headers()
                        while True:
                            chunk = recording_file.read(65536)
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                except ValueError as error:
                    self.send_json({'error': str(error)}, 409)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                except FileNotFoundError:
                    self.send_json({'error': '錄製檔案已不存在'}, 404)
                return
            if path == '/':
                body, kind = page, 'text/html; charset=utf-8'
            elif path in ('/api/snapshot', '/api/export'):
                payload = store.snapshot()
                payload['ros_domain_id'] = os.environ.get('ROS_DOMAIN_ID', '0')
                body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
                kind = 'application/json; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            if path == '/api/export':
                self.send_header('Content-Disposition', 'attachment; filename="rinbo-monitor.json"')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_args):
            pass

    return Handler


def create_monitor_node(store):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rinbo_msgs.msg import MotorCmdStamped, MotorStateStamped, PowerStateStamped
    from std_msgs.msg import Bool
    from rcl_interfaces.msg import Log
    from rosidl_runtime_py.convert import message_to_ordereddict

    node = Node('rinbo_monitor', enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    message_types = {'requested': MotorCmdStamped, 'forwarded': MotorCmdStamped,
                     'motor': MotorStateStamped, 'power': PowerStateStamped,
                     'output': Bool, 'ready': Bool, 'logs': Log}

    def receive(key, msg):
        if key == 'logs' and not msg.name.startswith(('rinbo_', 'redrhex_')):
            return
        store.ingest(key, message_to_ordereddict(msg))

    for key, msg_type in message_types.items():
        node.create_subscription(msg_type, TOPICS[key],
                                 lambda msg, key=key: receive(key, msg), qos)
    node.create_timer(0.1, store.sample_history)
    return node


def main(args=None):
    parser = argparse.ArgumentParser(description='Rinbo 唯讀即時監控網頁，不發送控制命令。')
    parser.add_argument('--host', default='127.0.0.1', help='用 0.0.0.0 允許 Windows 經區域網路查看')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--stale-seconds', type=float, default=0.5, help='畫面資料過期門檻，不改控制器保護')
    parser.add_argument('--record-dir', default=str(Path.cwd() / 'log' / 'monitor_recordings'),
                        help='手動錄製的 JSON 儲存目錄')
    options, ros_args = parser.parse_known_args(args)
    if not math.isfinite(options.stale_seconds) or options.stale_seconds <= 0:
        parser.error('--stale-seconds must be finite and positive')
    if not 1 <= options.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    import rclpy
    from rclpy.executors import ExternalShutdownException
    store = MonitorStore(stale_s=options.stale_seconds, recording_dir=options.record_dir)
    rclpy.init(args=ros_args)
    node = None
    server = None
    thread = None
    try:
        node = create_monitor_node(store)
        token = secrets.token_urlsafe(32)
        server = ThreadingHTTPServer((options.host, options.port), make_handler(store, token))
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print('唯讀監控：http://{}:{}  ROS_DOMAIN_ID={}'.format(
            options.host, options.port, os.environ.get('ROS_DOMAIN_ID', '0')), flush=True)
        print('存取連結：http://{}:{}/?token={}'.format(options.host, options.port, token), flush=True)
        if options.host == '0.0.0.0':
            print('Windows：將連結中的 0.0.0.0 換成平常連接 Orin 的 IP，保留 token。', flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server:
            if thread:
                server.shutdown()
                thread.join(timeout=2)
            server.server_close()
        if node:
            node.destroy_node()
        store.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
