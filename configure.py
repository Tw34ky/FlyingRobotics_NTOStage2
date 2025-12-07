import os
import shutil
import sys


def replace_string(file_path, old_string, new_string):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    for index, line in enumerate(lines):
        if old_string in line:
            lines[index] = new_string + '\n'
            break
    with open(file_path, 'w') as file:
        file.writelines(lines)

# Определение имени пользователя и пути
if len(sys.argv) > 1:
    if len(sys.argv) > 2:
        directory_name = ' '.join(sys.argv[1:])
    else:
        directory_name = sys.argv[1]
    user_name = directory_name.split('/')[0]
else:
    directory_name = 'clover/Desktop'
    user_name = 'clover'

# Изменение строк в launch файле для запуска ArUco
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/aruco.launch',
               '<arg name="aruco_detect"',
               ' <arg name="aruco_detect" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/aruco.launch',
               '<arg name="aruco_map"',
               ' <arg name="aruco_map" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/aruco.launch',
               '<arg name="aruco_vpe"',
               ' <arg name="aruco_vpe" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/aruco.launch',
               '<arg name="placement"',
               ' <arg name="placement" default="floor"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/aruco.launch',
               '<arg name="length"',
               ' <arg name="length" default="0.32"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/aruco.launch',
               '<arg name="map"',
               ' <arg name="map" default="cmit.txt"/>')

# Изменение строк в launch файле для запуска Clover
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="fcu_conn"',
               ' <arg name="fcu_conn" default="usb"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="fcu_ip"',
               ' <arg name="fcu_ip" default="127.0.0.1"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="fcu_sys_id"',
               ' <arg name="fcu_sys_id" default="1"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="gcs_bridge"',
               ' <arg name="gcs_bridge" default="tcp"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="web_video_server"',
               ' <arg name="web_video_server" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="rosbridge"',
               ' <arg name="rosbridge" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="main_camera"',
               ' <arg name="main_camera" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="optical_flow"',
               ' <arg name="optical_flow" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="aruco"',
               ' <arg name="aruco" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="rangefinder_vl53l1x"',
               ' <arg name="rangefinder_vl53l1x" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="led"',
               ' <arg name="led" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="blocks"',
               ' <arg name="blocks" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="rc"',
               ' <arg name="rc" default="false"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="force_init"',
               ' <arg name="force_init" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover/launch/clover.launch',
               '<arg name="simulator"',
               ' <arg name="simulator" default="false"/>')

# Изменение строк в launch файле для симулятора
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="type"',
               ' <arg name="type" default="gazebo"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="mav_id"',
               ' <arg name="mav_id" default="0"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="est"',
               ' <arg name="est" default="ekf2"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="vehicle"',
               ' <arg name="vehicle" default="clover"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="main_camera"',
               ' <arg name="main_camera" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="maintain_camera_rate"',
               ' <arg name="maintain_camera_rate" default="false"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="rangefinder"',
               ' <arg name="rangefinder" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="led"',
               ' <arg name="led" default="true"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="gps"',
               ' <arg name="gps" default="false"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="use_clover_physics"',
               ' <arg name="use_clover_physics" default="false"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="gui"',
               ' <arg name="gui" default="true"/>')
# replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
#                '<arg name="world_name" value="$(find clover_simulation)',
#                ' <arg name="world_name" value="$(find clover_simulation)/resources/worlds/Nto.world"/>')
replace_string(f'/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch',
               '<arg name="verbose"',
               ' <arg name="verbose" value="true"/>')

import texture

texture.main()


import generation

generation.main()