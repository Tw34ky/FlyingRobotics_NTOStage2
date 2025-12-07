import rospy
rospy.init_node('flight')
import math
import numpy as np
from clover import srv
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import requests as rq
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud
from cv_bridge import CvBridge
from clover import long_callback
from geometry_msgs.msg import Point
import cv2
from cv import process_image_with_optimal_lines
from img_to_coords import cnt_center


bridge = CvBridge()


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

last_coords = []
points = []
point_centers = []
TOLERANCE = 0.5
start = True
count = 0


publisher = rospy.Publisher("tubes", PointCloud, queue_size=10)
pub = rospy.Publisher('/pipeline_detection', Bool, queue_size=10)


def disambiguate_lines(line):
    """
    Disambiguate line direction based on image context and expected orientation.
    """

    rho, theta = line
    
    # Convert theta to degrees
    angle_degrees = np.degrees(theta) % 180
    
    # Adjust to -90 to 90 range for easier handling
    if angle_degrees > 90:
        angle_degrees -= 180
    
    # Calculate line endpoints for spatial analysis
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    
    # Calculate two points on the line (top and bottom of image)
    height = 240  # Assuming standard image height
    x1 = int(x0 + height * (-b))
    y1 = int(y0 + height * (a))
    x2 = int(x0 - height * (-b))
    y2 = int(y0 - height * (a))
    
    # Determine which endpoint is left/right based on x-coordinate
    left_x = x1 if x1 < x2 else x2
    right_x = x1 if x1 > x2 else x2
    
    # Calculate line direction based on spatial arrangement
    # If left endpoint is higher than right endpoint, line goes upward left-to-right
    left_y = y1 if x1 < x2 else y2
    right_y = y1 if x1 > x2 else y2
    
    # Determine the actual direction based on endpoint positions
    if left_y > right_y:  # Line goes downward left-to-right
        actual_direction = angle_degrees
    else:  # Line goes upward left-to-right
        # Flip the direction by 180 degrees
        actual_direction = angle_degrees + 180
        if actual_direction > 90:
            actual_direction -= 180
    
    # Ensure direction is within expected range (-30 to 30)
    if actual_direction < -30:
        actual_direction += 180
    elif actual_direction > 30:
        actual_direction -= 180
    
    # Only keep lines that are within expected direction range
    if -30 <= actual_direction <= 30:
        return rho, np.radians(actual_direction)
    


# When pipeline detected:


# When lost or transitioning:

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

pub.publish(Bool(False))
navigate_wait(z=1.0, frame_id='body', auto_arm=True)
rospy.sleep(2)
navigate_wait(x=1.0, y=1.0, z=1.0, yaw=0, frame_id='aruco_map')
pub.publish(Bool(True))
while not rospy.is_shutdown():
    
    rospy.sleep(1)
    data = rospy.wait_for_message('/main_camera/image_raw', Image)
    img = bridge.imgmsg_to_cv2(data, 'bgr8')


    mask, binary, lines, full_binary = process_image_with_optimal_lines(img)
    kernel_3 = np.ones((3, 3), np.uint8)
    kernel_5 = np.ones((5, 5), np.uint8)
    # Using cv2.erode() method a
    binary = cv2.erode(binary, kernel_5) 
    binary = cv2.dilate(binary, kernel_5) 
    cv2.imshow("bin", binary)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        M = cv2.moments(contours[0])
        try:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            setpoint, px, py = cnt_center(cx, cy, data)

            if start:
                telem = get_telemetry(frame_id='aruco_map') 
                angle = math.atan2(setpoint.point.y - telem.y, setpoint.point.x - telem.x)
                
                navigate_wait(x=setpoint.point.x, y=setpoint.point.y, z=1.0, yaw=angle, frame_id='aruco_map', tolerance=0.1)
                angle = 0

            navigate_wait(x=setpoint.point.x, y=setpoint.point.y, z=1.0, frame_id='aruco_map', tolerance=0.1) 
        except ZeroDivisionError:
            pass

    main_line = lines[0][0]
    main_line = disambiguate_lines(main_line)
    angle = lines[0][0][1]

    if abs(min(angle, abs(2 * math.pi - angle)) - math.pi) <= 0.1:
        angle = 0
    elif angle > math.pi / 2:
        angle = math.pi - angle
    else:
        angle = -angle

    if start:
        pub.publish(Bool(True))
        navigate(x=0.0, y=0.0, z=0.0, yaw=0, frame_id='body')
        rospy.sleep(3)
    else:
        navigate(x=0.0, y=0.0, z=0.0, yaw=angle, frame_id='body')
        rospy.sleep(3)
    
    kernel_7 = np.ones((7, 7), np.uint8)
    binary = cv2.dilate(binary, kernel_7)  
    binary = cv2.bitwise_and(binary, mask)

    cv2.imshow('bin1', binary)
    cv2.waitKey(1000)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        for contour in contours:

            M = cv2.moments(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if w <= h:
                try:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    setpoint, px, py = cnt_center(cx, cy, data)
                    flag = False
                    if len(points) != 0:
                        for point in points:
                            if math.sqrt((point.x - setpoint.point.x) ** 2 + (point.y - setpoint.point.y) ** 2) < TOLERANCE: 
                                flag = True
                                break

                    if not flag:
                        point_centers.append(setpoint.point)
                        points.append(setpoint.point)
                        points_msg = PointCloud()
                        points_msg.header.frame_id = "aruco_map"
                        points_msg.header.stamp = rospy.Time.now()
                        points_msg.points = points
                        publisher.publish(points_msg)

                except ZeroDivisionError:
                    pass    

    contours, _ = cv2.findContours(full_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        x, y, w, h = cv2.boundingRect(contours[0])
        if y > 100 and not start:
            print('MISSION IS SUPPOSED TO BE OVER')
            pub.publish(Bool(False))
            break

    if start:
        start = False
        continue

    navigate_wait(x=0.65, y=0.0, z=0.0, frame_id='body')

pub.publish(Bool(False))
navigate_wait(x=0.0, y=0.0, z=1.0, frame_id='aruco_map', speed=2)
navigate_wait(x=0.0, y=0.0, z=0.7, yaw=0.0, frame_id='aruco_map', speed=1, tolerance=0.1)

land()
rospy.sleep(5)
