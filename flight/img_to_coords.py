import rospy
import cv2
import tf2_ros
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from clover import long_callback, srv
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
import tf2_geometry_msgs

camera_info = rospy.wait_for_message('/main_camera/camera_info', CameraInfo)
camera_matrix = np.float64(camera_info.K).reshape(3, 3)
distortion = np.float64(camera_info.D).flatten()
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)


def img_xy_to_point(xy, dist):
    xy = cv2.undistortPoints(xy, camera_matrix, distortion, P=camera_matrix)[0][0]

    # Shift points to center
    xy -= camera_info.width // 2, camera_info.height // 2

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]

    return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)


def cnt_center(cx, cy, data):    # data is img message
    altitude = get_telemetry('terrain').z
    # cx += 20
    # cy += 20
    xy3d = img_xy_to_point((cx, cy), altitude)
    target = PointStamped(header=data.header, point=xy3d)
    setpoint = tf_buffer.transform(target, 'aruco_map', timeout=rospy.Duration(0.2))
    return setpoint, cx-20, cy-20
