from flask import Flask, render_template, jsonify, request
import rospy
from clover import srv
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import threading
import time
from aruco_pose.msg import MarkerArray
import requests as rq
import logging


log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


app = Flask(__name__, static_folder='static')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
drone_state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
taps = []
mission_status = 'idle'  # idle | flying | aborted | killed
search_status = False
coords = []
proc = None

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


@app.route('/map')
def get_map():
    try:
        # Wait for latest marker array
        msg = rospy.wait_for_message('/aruco_map/map', MarkerArray, timeout=2.0)
        markers = []
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z
            markers.append({
                "id": marker.id,
                "x": x,
                "y": y,
                "z": z,
                "length": marker.length  # side length of square marker
            })
        return jsonify({"markers": markers})
    except Exception as e:
        print("Error fetching map:", e)
        return jsonify({"markers": []})


def search_status_callback(msg):
    global search_status
    search_status = msg.data


def drone_pose_callback(msg):
    drone_state['x'] = msg.pose.position.x
    drone_state['y'] = msg.pose.position.y
    # Yaw из кватерниона
    from tf.transformations import euler_from_quaternion
    q = msg.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    drone_state['yaw'] = yaw


def tubes_callback(msg):
    global taps

    new_taps = []
    for i in range(0, len(msg.points)):
        new_taps.append({'x': msg.points[i].x, 'y': msg.points[i].y})
    taps = new_taps


@app.route('/')
def index():
    return render_template('index.html')

# API
@app.route('/drone')
def get_drone():
    global search_status
    telem = get_telemetry(frame_id='aruco_map')
    drone_state = {'x': telem.x, 'y': telem.y, 'yaw': telem.yaw}
    if search_status:
        coords.append(drone_state)
    return jsonify(drone_state)


@app.route('/route')
def get_route():
    return coords


@app.route('/taps')
def get_taps():
    global taps
    return jsonify(taps)


@app.route('/status')
def get_status():
    return jsonify({'status': mission_status})

@app.route('/command', methods=['POST'])
def post_command():
    global mission_status
    cmd = request.json.get('command')
    if cmd == 'start' and mission_status == 'idle':
        mission_status = 'flying'
        def run_mission():
            import subprocess, platform
            plat = platform.system()
            if plat == 'Linux' or plat == 'Darwin':
                runner = 'python3'
            else:
                runner = 'python'
            global proc
            proc = subprocess.Popen([runner, "flight/flight.py"], shell=False)

            global mission_status
            if mission_status == 'flying':
                mission_status = 'idle'
        run_mission()
    elif cmd == 'stop':
        try:
            proc.kill()
        except:
            print('?')
            pass
        navigate_wait(x=0.0, y=0.0, z=0.0, frame_id='aruco_map')

    elif cmd == 'kill':
        try:
            from mavros_msgs.srv import CommandBool
            arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
            set_attitude(thrust=0)
            arming(False)
            mission_status = 'killed'
        except Exception as e:
            print(e)
            
    return jsonify({'status': 'ok'})

def ros_listener():
    rospy.init_node('web_bridge', anonymous=True, disable_signals=True)
    rospy.Subscriber('get_telemetry', PoseStamped, drone_pose_callback)
    rospy.Subscriber('/tubes', PointCloud, tubes_callback)
    rospy.Subscriber('/pipeline_detection', Bool, search_status_callback)
    rospy.spin()

if __name__ == '__main__':
    threading.Thread(target=ros_listener, daemon=True).start()
    app.run(host='127.0.10.10', port=5000, debug=False)