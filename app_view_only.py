from flask import Flask, render_template, Response, jsonify
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import psutil

app = Flask(__name__, template_folder="templates", static_folder="static")

frames = {
    "cam1": None,
    "cam2": None,
    "cam3": None,
    "cam4": None
}

locks = {
    "cam1": threading.Lock(),
    "cam2": threading.Lock(),
    "cam3": threading.Lock(),
    "cam4": threading.Lock()
}

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_flask_subscriber')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(CompressedImage, '/camera1', lambda m: self.cb(m, "cam1"), qos)
        self.create_subscription(CompressedImage, '/camera2', lambda m: self.cb(m, "cam2"), qos)
        self.create_subscription(CompressedImage, '/camera3', lambda m: self.cb(m, "cam3"), qos)
        self.create_subscription(CompressedImage, '/camera4', lambda m: self.cb(m, "cam4"), qos)

    def cb(self, msg, cam):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        with locks[cam]:
            frames[cam] = frame

def ros_spin():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def stream(cam):
    while True:
        with locks[cam]:
            if frames[cam] is None:
                time.sleep(0.01)
                continue
            frame = frames[cam].copy()

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               buffer.tobytes() +
               b'\r\n')

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video/cam1")
def cam1():
    return Response(stream("cam1"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/video/cam2")
def cam2():
    return Response(stream("cam2"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/video/cam3")
def cam3():
    return Response(stream("cam3"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/video/cam4")
def cam4():
    return Response(stream("cam4"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/system_stats")
def system_stats():
    cpu = psutil.cpu_percent(interval=0.1)
    mem = psutil.virtual_memory().percent
    temp = 0.0
    temps = psutil.sensors_temperatures()
    if temps:
        for _, entries in temps.items():
            if entries:
                temp = entries[0].current
                break
    return jsonify({
        "cpu": cpu,
        "memory": mem,
        "temperature": temp
    })

if __name__ == "__main__":
    threading.Thread(target=ros_spin, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

