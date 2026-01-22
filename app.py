import os
import time
import threading
import math  # for yaw calc

from flask import Flask, render_template
import eventlet
import eventlet.wsgi
import socketio
import psutil

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_from_quaternion  # pip install transforms3d

# Flask + Socket.IO setup
sio = socketio.Server(cors_allowed_origins='*')
app = Flask(__name__, template_folder='templates', static_folder='static')
flask_app = socketio.WSGIApp(sio, app)

@app.route('/')
def index():
    return render_template('index.html')

# ROS2 bridge node
class RoverBridgeNode(Node):
    def __init__(self):
        super().__init__('rover_bridge_node')
        
        # cmd_vel publisher for rover control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # topic subscribers - add your actual topics here
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        # fake battery/temp for demo - replace with real topics
        self.create_subscription(std_msgs.msg.Float64, '/battery_voltage', self.battery_callback, 10)
        self.create_subscription(std_msgs.msg.Float64, '/temperature', self.temp_callback, 10)
        
        # data cache
        self.odom_data = None
        self.gps_data = None
        self.sensors = {'battery_voltage': 12.6, 'temperature': 25.3}
        self.last_emit = 0.0

    def odom_callback(self, msg):
        self.odom_data = msg
        self.emit_state()

    def gps_callback(self, msg):
        self.gps_data = msg
        self.emit_state()

    def battery_callback(self, msg):
        self.sensors['battery_voltage'] = msg.data

    def temp_callback(self, msg):
        self.sensors['temperature'] = msg.data

    def emit_state(self):
        now = time.time()
        if now - self.last_emit < 0.05:  # 20Hz max
            return
        self.last_emit = now

        data = {}

        # extract odom with yaw
        if self.odom_data:
            pose = self.odom_data.pose.pose
            twist = self.odom_data.twist.twist
            
            # yaw from quaternion
            quat = [pose.orientation.x, pose.orientation.y, 
                   pose.orientation.z, pose.orientation.w]
            yaw = euler_from_quaternion(quat)[2]  # roll pitch yaw
            
            data['odometry'] = {
                'x': pose.position.x,
                'y': pose.position.y,
                'theta': yaw,
                'velocity': twist.linear.x
            }

        # gps
        if self.gps_data:
            data['gps'] = {
                'latitude': self.gps_data.latitude,
                'longitude': self.gps_data.longitude
            }

        # sensors for overview cards
        data['sensors'] = self.sensors

        sio.emit('ros2_topic_update', data)

    def publish_cmd(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'cmd_vel: v={linear_vel:.2f} w={angular_vel:.2f}')

# global ros node
ros_node = None

# socketio events
@sio.event
def connect(sid, environ):
    print(f'🎮 Web client {sid} connected')

@sio.event
def disconnect(sid):
    print(f'🔌 Web client {sid} disconnected')

@sio.on('send_command')
def handle_cmd(sid, data):
    global ros_node
    cmd = data.get('command')
    v = data.get('velocity', 0.0)
    w = data.get('angular_velocity', 0.0)

    print(f'📡 {sid}: {cmd} v={v:.2f} w={w:.2f}')

    if ros_node is None:
        print('❌ ROS node not ready')
        return

    # map web buttons to rover actions
    if cmd == 'start':
        ros_node.publish_cmd(v, w)
    elif cmd == 'stop':
        ros_node.publish_cmd(0.0, 0.0)
    elif cmd == 'arm':
        print('🦾 ARM command (add service call)')
    elif cmd == 'disarm':
        print('🔓 DISARM command (add service call)')
    elif cmd == 'home':
        print('🏠 HOME command (add action)')
    elif cmd == 'reset':
        print('🔄 RESET command')

# threads
def ros_spin():
    global ros_node
    rclpy.init()
    ros_node = RoverBridgeNode()
    print('🚀 ROS2 bridge ready')
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

def push_stats():
    """Push CPU/mem stats every second"""
    while True:
        try:
            cpu = psutil.cpu_percent(interval=0.1)
            mem = psutil.virtual_memory().percent
            nodes = 1 if ros_node else 0
            
            stats = {
                'cpu': cpu,
                'memory': mem,
                'latency': 12,  # fake
                'active_nodes': nodes
            }
            sio.emit('system_stats', stats)
        except:
            pass
        time.sleep(1)

# main
if __name__ == '__main__':
    print('🌐 Starting ROS2 Rover Monitor...')
    
    # start ROS2
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # stats thread
    stats_thread = threading.Thread(target=push_stats, daemon=True)
    stats_thread.start()
    
    # web server
    port = int(os.environ.get('PORT', 5000))
    print(f'🌍 UI ready at http://localhost:{port}')
    print('📡 SocketIO on /socket.io/')
    print('🔗 Open browser now!')
    
    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', port)), flask_app)
