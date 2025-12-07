from flask import Flask, render_template, jsonify, request
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped
import threading
import time

app = Flask(__name__, static_folder='static')

# Shared state
drone_state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
taps = []
mission_status = 'idle'  # idle | flying | aborted | killed

def drone_pose_callback(msg):
    # Предполагаем, что данные приходят в frame_id='aruco_map'
    drone_state['x'] = msg.pose.position.x
    drone_state['y'] = msg.pose.position.y
    # Yaw из кватерниона
    from tf.transformations import euler_from_quaternion
    q = msg.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    drone_state['yaw'] = yaw

def tubes_callback(msg):
    # msg.data = [x1, y1, z1, x2, y2, z2, ...]
    global taps
    data = msg.data
    new_taps = []
    for i in range(0, len(data), 3):
        if i+2 < len(data):
            x, y = data[i], data[i+1]
            # Фильтр дубликатов
            duplicate = any(abs(x-t[0])<0.3 and abs(y-t[1])<0.3 for t in taps)
            if not duplicate:
                new_taps.append({'x': x, 'y': y})
    taps.extend(new_taps)

@app.route('/')
def index():
    return render_template('index.html')

# API
@app.route('/drone')
def get_drone():
    return jsonify(drone_state)

@app.route('/taps')
def get_taps():
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
            import subprocess, sys
            try:
                subprocess.run([sys.executable, '-m', 'roslaunch', 'pipeline_monitoring', 'mission.launch'], 
                               cwd='/path/to/your/catkin_ws',  # укажите свой путь
                               timeout=300)
            except:
                pass
            global mission_status
            if mission_status == 'flying':
                mission_status = 'idle'
        threading.Thread(target=run_mission, daemon=True).start()
    elif cmd == 'stop':
        try:
            rospy.wait_for_service('/land', timeout=1)
            from std_srvs.srv import Trigger
            land = rospy.ServiceProxy('/land', Trigger)
            land()
            mission_status = 'idle'
        except:
            pass
    elif cmd == 'kill':
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=1)
            from mavros_msgs.srv import CommandBool
            arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm(False)
            mission_status = 'killed'
        except:
            pass
    return jsonify({'status': 'ok'})

def ros_listener():
    rospy.init_node('web_bridge', anonymous=True)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, drone_pose_callback)
    rospy.Subscriber('/tubes', PointCloud, tubes_callback)
    rospy.spin()

if __name__ == '__main__':
    threading.Thread(target=ros_listener, daemon=True).start()
    app.run(host='127.0.0.10', port=5000, debug=False)