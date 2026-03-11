from flask import Flask, render_template, jsonify, request, send_file
import rospy
from clover import srv
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import threading
import numpy as np
import subprocess
import time, math, os, json, io, base64

from sensor_msgs.msg import Image
import logging

import platform
from cloverAPI import CloverAPI
from aruco_pose.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError


app = Flask(__name__)

plat = platform.system()
if plat == 'Linux' or plat == 'Darwin':
    runner = 'python3'
else:
    runner = 'python'


log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


app = Flask(__name__, static_folder='static')
W, H = 800, 800          # Размер изображения
MARGIN = 40  

drone_state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
taps = []
mission_status = 'idle'  # idle | flying | aborted | killed
search_status = False
coords = []
proc = None

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)

# Кэш: хэш маркеров -> (base64_image, metadata)
_map_cache = {}


def calculate_image_position() -> 'tuple[float, float, float, float]':
    """
    Ваша функция: вычисление позиции и масштаба изображения карты.
    Возвращает: (x, y, lx, ly) — смещение и размеры в мировых координатах.
    """
    aruco_map = get_markers_from_ros()  # Получаем маркеры из ROS
    if not aruco_map:
        return 0, 0, W, H
    
    left = min(aruco_map, key=lambda x: x["x"] - x["length"] / 2)
    right = max(aruco_map, key=lambda x: x["x"] + x["length"] / 2)
    top = max(aruco_map, key=lambda x: x["y"] + x["length"] / 2)
    bottom = min(aruco_map, key=lambda x: x["y"] - x["length"] / 2)
    dx = right["x"] - left["x"]
    dy = top["y"] - bottom["y"]
    
    if dx >= dy:
        k1, k2 = right["length"] / dx, left["length"] / dx
        dx_px = (2 * W - 4 * MARGIN) / (2 + k1 + k2)
        scale = dx / dx_px
        margin_y = (H - (W - 2 * MARGIN) * (dy + (top["length"] + bottom["length"]) / 2) / (dx + (right["length"] + left["length"]) / 2)) / 2
        y = top["y"] + margin_y * scale + top["length"] / 2
        x = left["x"] - MARGIN * scale - left["length"] / 2
        ly, lx = H * scale, W * scale
    else:
        k1, k2 = top["length"] / dy, bottom["length"] / dy  # Исправлено: было dx, должно быть dy
        dy_px = (2 * H - 4 * MARGIN) / (2 + k1 + k2)
        scale = dy / dy_px
        margin_x = (W - (H - 2 * MARGIN) * (dx + (right["length"] + left["length"]) / 2) / (dy + (top["length"] + bottom["length"]) / 2)) / 2
        y = top["y"] + MARGIN * scale + top["length"] / 2
        x = left["x"] - margin_x * scale - left["length"] / 2
        ly, lx = H * scale, W * scale
    return x, y, lx, ly


def get_markers_from_ros():
    """Получает маркеры из топика /aruco_map/map"""
    try:
        msg = rospy.wait_for_message('/aruco_map/map', MarkerArray)
        markers = []
        for m in msg.markers:
            markers.append({
                'id': m.id,
                'x': m.pose.position.x,
                'y': m.pose.position.y,
                'z': 0,
                'length': m.length
            })
        return markers
    except Exception as e:
        print('get markers :', e)
        return []


def get_map_image_from_ros():
    """Получает изображение карты из топика /aruco_map/image"""
    try:
        img_msg = rospy.wait_for_message('/aruco_map/image', Image)
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # Конвертируем BGR -> RGB для PIL/base64
        rgb_image = cv_image[:, :, ::-1]
        return rgb_image
    except CvBridgeError as e:
        app.logger.error(f"CvBridge error: {e}")
        return None
    except Exception as e:
        app.logger.error(f"Error getting map image: {e}")
        return None


def image_to_base64(cv_image_rgb):
    """Конвертирует numpy array (RGB) в base64-строку"""
    from PIL import Image
    img = Image.fromarray(cv_image_rgb)
    buffer = io.BytesIO()
    img.save(buffer, format='PNG', optimize=True)
    buffer.seek(0)
    return base64.b64encode(buffer.read()).decode('utf-8')


def get_map_data():
    """
    Получает изображение карты + маркеры + вычисляет метаданные.
    Центрирует первый маркер в (0, 0).
    """
    # Получаем данные
    markers = get_markers_from_ros()
    if not markers: 
        print('not markers')
    map_image = get_map_image_from_ros()
    
    try:
        if not map_image:
            print('not map')
            return None
    except:
        pass
        
    # Вычисляем позицию и масштаб через вашу функцию
    x_offset, y_offset, width_world, height_world = calculate_image_position()
    

    # Если есть маркеры — центрируем на первом
    center_marker = markers[0] if markers else None
    if center_marker:
        # Сдвиг координат: чтобы первый маркер оказался в (0, 0)
        shift_x = -center_marker['x']
        shift_y = -center_marker['y']
        x_offset += shift_x
        y_offset += shift_y
    x_offset = -0.72
    y_offset = -1.69

    print(width_world, height_world)
    k = 0.7
    width_world += k
    height_world += k
    
    # Метаданные для фронтенда
    metadata = {
        'offset_x': x_offset,           # Смещение левого края текстуры (мировые координаты)
        'offset_y': y_offset,           # Смещение верхнего края текстуры
        'width_world': width_world,     # Ширина текстуры в метрах
        'height_world': height_world,   # Высота текстуры в метрах
        'center_marker_id': center_marker['id'] if center_marker else None,
        'markers_count': len(markers)
    }
    
    # Конвертируем изображение в base64
    img_base64 = image_to_base64(map_image)
    # print(img_base64)
    return {
        'image': f'data:image/png;base64,{img_base64}',
        'metadata': metadata,
        'markers': markers
    }


@app.before_request
def get_clover_instance():
    global clover, bridge
    # rospy.init_node('')

    clover = CloverAPI()    
    bridge = CvBridge()


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


@app.route('/config/buildings.json')
def get_buildings_config():
    """Отдаёт JSON-конфиг зданий"""
    try:
        config_path = os.path.join(app.static_folder, 'config', 'buildings.json')
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        return jsonify(config)
    except FileNotFoundError:
        # Возвращаем пустой конфиг, если файл не найден
        return jsonify({"buildings": [], "metadata": {}})
    except Exception as e:
        app.logger.error(f"Error loading buildings config: {e}")
        return jsonify({"error": str(e)}), 500


@app.route('/map')
def get_map():
    print('map called')
    """Отдаёт карту: изображение + метаданные + маркеры"""
    result = get_map_data()
    
    if result is None:
        return jsonify({
            'error': 'Failed to get map from ROS',
            'markers': [],
            'texture': None,
            'texture_meta': None
        }), 503
    
    return jsonify({
        'markers': result['markers'],
        'source': 'ros',
        'count': len(result['markers']),
        'texture': result['image'],
        'texture_meta': result['metadata']
    })


@app.route('/drone')
def get_drone():
    global search_status, drone_state
    try:
        # Запрос телеметрии
        telem = get_telemetry(frame_id='aruco_map')
        print(telem)
        
        if not telem.x or telem.x == np.nan or math.isnan(telem.x):
            telem.x = 0
        
        if not telem.y or telem.y == np.nan or math.isnan(telem.y) :
            telem.y = 0
        
        if not telem.z or telem.z == np.nan or math.isnan(telem.z) :
            telem.z = 0

        if not telem.yaw or telem.yaw == np.nan or math.isnan(telem.yaw) :
            telem.yaw = 0

        # Формирование ответа
        drone_state = {
            'x': telem.x, 
            'y': telem.y, 
            'z': telem.z,      # Добавляем высоту (обычно ~0.5-1.0м)
            'yaw': telem.yaw   # Угол поворота в радианах
        }
        
        # # Логика записи маршрута
        # if search_status:
        #     coords.append(drone_state)
        print(drone_state)
        return jsonify(drone_state)
    except Exception as e:
        # Возвращаем заглушку при ошибке, чтобы фронтенд не падал
        return jsonify({'x': 0, 'y': 0, 'z': 0.5, 'yaw': 0})


@app.route('/state', methods=['GET'])
def get_state():
    try:
        bat = clover.get_battery_state()
        # mavros = clover.get_state()
        return jsonify({
            "voltage": bat.voltage, 
            "percentage": bat.percentage
        })
    except rospy.ROSException as e:
        return jsonify({"error": str(e)}), 500


@app.route('/map/texture.png')
def get_map_texture():
    print('map fetched')
    """Прямая отдача PNG-изображения карты"""
    map_image = get_map_image_from_ros()
    if map_image is None:
        return jsonify({'error': 'No map image available'}), 503
    
    return send_file(
        io.BytesIO(base64.b64decode(image_to_base64(map_image))),
        mimetype='image/png',
        download_name='map.png'
    )


def search_status_callback(msg):
    global search_status
    search_status = msg.data


def drone_pose_callback(msg):
    global drone_state
    drone_state['x'] = msg.pose.position.x
    drone_state['y'] = msg.pose.position.y
    drone_state['z'] = msg.pose.position.z
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
    return render_template('index2.html')


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
    
    print(cmd, mission_status)

    if cmd == 'start' and mission_status != 'flying':
        mission_status = 'flying'
        
        global proc, runner
        
        proc = subprocess.Popen([runner, "flight/flight.py"], shell=False)

    elif cmd == 'stop':
        mission_status = 'interrupted'
        try:
            if proc is not None:  # Проверяем, существует ли процесс
                proc.kill()
                proc = None       # Сбрасываем ссылку
            clover.navigate(x=0, y=0, z=0, frame_id='body')
        except Exception as e:
            print(f"Error stopping process: {e}")
            pass

    elif cmd == 'home':

        # Остановка процесса полета 
        try:
            if proc is not None:  # Проверяем, существует ли процесс
                proc.kill()
                proc = None       # Сбрасываем ссылку
        except Exception as e:
            print(f"Error stopping process: {e}")
            pass

        navigate_wait(x=0.0, y=0.0, z=0.0, frame_id='aruco_map') # возвращение на базу

        mission_status = 'idle' 

    elif cmd == 'kill':

        # Килл свитч

        try:
            from mavros_msgs.srv import CommandBool
            arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
            set_attitude(thrust=0)
            arming(False)
            mission_status = 'killed'
        except Exception as e:
            print(e)
            
    elif cmd == 'land' and mission_status == 'interrupted':
        clover.land() # Приземление 
        mission_status = 'idle'

    return jsonify({'status': 'ok'})


def ros_listener():
    rospy.init_node('web_bridge', anonymous=True, disable_signals=True)
    rospy.Subscriber('get_telemetry', PoseStamped, drone_pose_callback)
    rospy.Subscriber('/tubes', PointCloud, tubes_callback)
    rospy.Subscriber('/pipeline_detection', Bool, search_status_callback)
    rospy.spin()


def monitor_process():
    global proc, mission_status
    while True:
        # Если процесс запущен
        if proc is not None:
            # poll() возвращает None, если процесс еще работает, 
            # и код завершения, если процесс завершился
            if proc.poll() is not None:
                print("Mission process finished.")
                mission_status = 'idle'
                proc = None  # Сбрасываем ссылку, чтобы не проверять старый процесс
        time.sleep(0.5)  # Проверка каждые полсекунды


if __name__ == '__main__':
    threading.Thread(target=ros_listener, daemon=True).start()

    threading.Thread(target=monitor_process, daemon=True).start()
    
    app.run(host='127.0.10.10', port=5000, debug=True)
