import sys, os, shutil, random, math, getpass
from typing import *


PLACE_COUNT_GENERAL = 0 # нужен для создания названий моделей
MAIN_LINE_COUNT = 0 # для определения, когда делать отклонение основной линии
START_COORDS = (1, 1) # плейсхолдер для первой линии
SUM_LENGTH = None # заполнится при запуске рандомным значением
HEIGHT_ABOVE_FLOOR = 0.05
LINE_WIDTH = 0.2


def generate_coords(line_type: str):
    global MAIN_LINE_COUNT, START_COORDS, SUM_LENGTH

    """
    
    Два типа координат - для врезок ('cut') и для основной линии ('line')

    """

    if line_type == 'line':

        line_length = round(random.uniform(5, 10), 2)
        
        line_angle = random.randint(0, 90)
        line_angle = round(line_angle * math.pi / 180, 5)
        end_coords = (line_length * math.cos(line_angle) + START_COORDS[0], line_length * math.sin(line_angle) + START_COORDS[1])

        cx, cy = (START_COORDS[0] + end_coords[0]) / 2, (START_COORDS[1] + end_coords[1]) / 2
        print(START_COORDS, end_coords, line_angle)
        START_COORDS = end_coords
        line_pose = (cx, cy, HEIGHT_ABOVE_FLOOR, 0, 0, line_angle)

        line_size = (line_length, LINE_WIDTH, 0.001)
        place_line(line_pose, line_size)
        

def place_line(pose: Tuple, size: Tuple):
    global PLACE_COUNT_GENERAL
    """

    Кортеж pose состоит из x, y, z, roll, pitch, yaw

    z - высота

    """
    x, y, z, roll, pitch, yaw = pose

    pose_string = ' '.join(map(str, (x, y, z, roll, pitch, yaw)))
    size_string = ' '.join(map(str, size))
    template = f"""    <model name="line_{PLACE_COUNT_GENERAL}">
        <static>true</static>
        <link name="square_link_{PLACE_COUNT_GENERAL}">
            <pose>{pose_string}</pose>
            <visual name="square_texture">
                <cast_shadows>false</cast_shadows>
                <geometry>
                    <box>
                        <size>{size_string}</size>
                    </box>
                </geometry>
                <material>
                    <script>
                        <uri>model://square_line/materials/scripts</uri>
                        <uri>model://square_line/materials/textures</uri>
                        <name>square_solid</name>
                    </script>
                </material>
            </visual>
        </link>
    </model>
    """
    PLACE_COUNT_GENERAL += 1
    abs_path = f'/home/{getpass.getuser()}/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world'
    insert_line(abs_path, template)
    

def insert_line(file_path, template):
    """
    
    Читает кфг файл мира, добавляет в него линии, function's name is kind of self-explanatory

    """

    with open(file_path, 'r+') as file:
        offsets = []
        offset = 0
        contents = ''
        for index, content in enumerate(file.readlines()):
            if '<include>' in content:
                break
            offsets.append(offset)
            offset += len(content)
            contents += content
        saved_index = index
        contents += '\n'
        contents += template
        contents += '\n'
    with open(file_path, 'r+') as file:
        for index, content in enumerate(file.readlines()[saved_index::]):
            offsets.append(offset)
            offset += len(content)
            contents += content

    with open(file_path, 'w+') as file:
        file.write(contents)
            
# insert_line(f'/home/{getpass.getuser()}/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world')            

def main():
    sum_length = random.randint(5, 10)
    for i in range(2):
        generate_coords('line')

if __name__ == '__main__':
    main()