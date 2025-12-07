import rospy
rospy.init_node('flight')
import math
import numpy as np
from clover import srv
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

# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# fourcc2 = cv2.VideoWriter_fourcc(*'XVID')

# out_main = cv2.VideoWriter('main_camera.avi', fourcc, 30.0, (320,240))
# out_thermal = cv2.VideoWriter('thermal.avi', fourcc2, 25.0, (256,192))
bridge = CvBridge()

# @long_callback
# def image_callback(data):
#     img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

# @long_callback
# def thermal_callback(data):
#     img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
#     out_thermal.write(img)

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


# def heat_pipe() -> dict:
#     data = rq.get()
publisher = rospy.Publisher("tubes", PointCloud, queue_size=10)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


navigate_wait(z=1.0, frame_id='body', auto_arm=True)
rospy.sleep(2)
navigate_wait(x=1.0, y=1.0, z=1.0, yaw=0, frame_id='aruco_map')

while not rospy.is_shutdown():
    rospy.sleep(1)
    data = rospy.wait_for_message('/main_camera/image_raw', Image)
    img = bridge.imgmsg_to_cv2(data, 'bgr8')


    mask, binary, lines, full_binary = process_image_with_optimal_lines(img)
    kernel_3 = np.ones((3, 3), np.uint8)

    # Using cv2.erode() method a
    binary = cv2.erode(binary, kernel_3) 
    print(lines)
    cv2.imshow('bin', binary)  
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        M = cv2.moments(contours[0])
        try:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            setpoint, px, py = cnt_center(cx, cy, data)
            # if cy > 180:
            #     print('MISSION IS SUPPOSED TO BE OVER')
            #     break
            # print(setpoint.point.x, setpoint.point.y)
            if start:
                # navigate_wait(x=-0.5, y=-0.5, z=0.0, yaw=angle, frame_id='body', tolerance=0.1)

                telem = get_telemetry(frame_id='aruco_map')
                angle = math.atan2(setpoint.point.y - telem.y, setpoint.point.x - telem.x)
                # angle += math.pi * 1.5
                # print(angle)
                navigate_wait(x=setpoint.point.x, y=setpoint.point.y, z=1.0, yaw=angle, frame_id='aruco_map', tolerance=0.1)
                angle = 0

            navigate_wait(x=setpoint.point.x, y=setpoint.point.y, z=1.0, frame_id='aruco_map', tolerance=0.1) 
        except ZeroDivisionError:
            pass

    contours, _ = cv2.findContours(full_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        M = cv2.moments(contours[0])
        x, y, w, h = cv2.boundingRect(contours[0])
        if y > 100:
            print('MISSION IS SUPPOSED TO BE OVER', 'BINARY SEARCH')
            break
        # try:
        #     cx = int(M['m10']/M['m00'])
        #     cy = int(M['m01']/M['m00'])
        #     setpoint, px, py = cnt_center(cx, cy, data)
        #     if cy > 190:
        #         print('MISSION IS SUPPOSED TO BE OVER', 'BINARY SEARCH')
        #         break
        # except ZeroDivisionError:
        #     pass    


    angle = lines[0][0][1]

    print(angle)
    if abs(min(angle, abs(2 * math.pi - angle)) - math.pi) <= 0.2:
        angle = 0
    elif angle > 0.5:
        angle -= math.pi
    elif angle <= math.pi:
        pass
    else:
        angle -= math.pi

    if start:
        navigate(x=0.0, y=0.0, z=0.0, yaw=0, frame_id='body')
        rospy.sleep(3)
    else:
        navigate(x=0.0, y=0.0, z=0.0, yaw=-angle, frame_id='body')
        rospy.sleep(3)

    # contours_align, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)    

    kernel_7 = np.ones((7, 7), np.uint8)
    binary = cv2.dilate(binary, kernel_7)  
    binary = cv2.bitwise_and(binary, mask)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        for contour in contours:

            M = cv2.moments(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if w <= h: # and 0.01 < h / w < 0.1:
                try:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # cx += min(lines[0][0][0] - cx, lines[1][0][0] - cx, key=abs)
                    print(w / h, h / w)
                    setpoint, px, py = cnt_center(cx, cy, data)
                    flag = False
                    if len(points) != 0:
                        for point in points:
                            if math.sqrt((point.x - setpoint.point.x) ** 2 + (point.y - setpoint.point.y) ** 2) < TOLERANCE: 
                                flag = True
                                break

                    if not flag and angle < 0.1:
                        print(setpoint.point.x, setpoint.point.y, angle)
                        point_centers.append(setpoint.point)
                        points.append(setpoint.point)
                        points_msg = PointCloud()
                        points_msg.header.frame_id = "aruco_map"
                        points_msg.header.stamp = rospy.Time.now()
                        points_msg.points = points
                        publisher.publish(points_msg)


                        # cv2.drawContours(img, contours, 0, (255,255,0), 2, cv2.LINE_AA)
                        # cv2.circle(img, center=(int(cx), cy), radius=5, color=(255, 0, 255), thickness=-1)
                except ZeroDivisionError:
                    print('ZeroDivisionError')
                    pass    

    cv2.imshow("lolol", binary)
    cv2.waitKey(1000)
    if start:
        start = False
        continue
    navigate_wait(x=0.8, y=0.0, z=0.0, frame_id='body')
    print()

# image_sub = rospy.Subscriber('/main_camera/image_raw', Image, image_callback)
# thermal_sub = rospy.Subscriber('/cv/debug', Image, thermal_callback)
# rospy.sleep(5)
navigate_wait(x=0.0, y=0.0, z=1.0, frame_id='aruco_map', speed=2)
navigate_wait(x=0.0, y=0.0, z=0.7, yaw=0.0, frame_id='aruco_map', speed=1, tolerance=0.1)

land()
rospy.sleep(5)

# rospy.spin()
