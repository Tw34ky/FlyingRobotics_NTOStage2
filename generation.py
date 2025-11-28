import sys, os, shutil, random, math, getpass
import numpy as np
from typing import *


PLACE_COUNT_GENERAL = 0 # нужен для создания названий моделей
MAIN_LINE_COUNT = 0 # для определения, когда делать отклонение основной линии
START_COORDS = [1, 1] # плейсхолдер для первой линии
SUM_LENGTH = None # заполнится при запуске рандомным значением
HEIGHT_ABOVE_FLOOR = 0.05
LINE_WIDTH = 0.2
CUT_WIDTH = 0.1
LINE_COORDS = [] # здесь будут содержаться стартовыые и конечные координаты созданных линий 
LINE_COORDS_UNSPLIT = []


def generate_pipeline_insertions(line_coords, num_insertions=5, min_distance=0.75, branch_length=2.0):
    """
    Генерирует врезки (ответвления) от основной трубы нефтепровода
    
    Args:
        line_coords: список координат основной трубы [(x1,y1), (x2,y2), ...]
        num_insertions: количество врезок (по умолчанию 5)
        min_distance: минимальное расстояние между врезками (по умолчанию 0.75 м)
        branch_length: длина ответвления (по умолчанию 2.0 м)
    
    Returns:
        list: список словарей с информацией о врезках
    """
    
    def calculate_line_length(segment):
        """Вычисляет длину сегмента трубы"""
        (x1, y1), (x2, y2) = segment
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def calculate_angle(segment):
        """Вычисляет угол наклона сегмента трубы в радианах"""
        (x1, y1), (x2, y2) = segment
        return math.atan2(y2 - y1, x2 - x1)
    
    def point_on_segment(segment, distance_from_start):
        """Находит точку на сегменте на заданном расстоянии от начала"""
        (x1, y1), (x2, y2) = segment
        segment_length = calculate_line_length(segment)
        
        if segment_length == 0:
            return (x1, y1)
        
        t = distance_from_start / segment_length
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        
        return (x, y)
    
    def calculate_distance(point1, point2):
        """Вычисляет расстояние между двумя точками"""
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Собираем все сегменты трубы
    segments = []
    total_length = 0
    segment_lengths = []
    
    for i in range(len(line_coords) - 1):
        segment = (line_coords[i], line_coords[i + 1])
        segments.append(segment)
        segment_length = calculate_line_length(segment)
        segment_lengths.append(segment_length)
        total_length += segment_length
    
    # Проверяем, достаточно ли длины трубы для всех врезок
    required_length = (num_insertions - 1) * min_distance
    if total_length < required_length:
        raise ValueError(f"Длина трубы ({total_length:.2f} м) недостаточна для размещения {num_insertions} врезок с минимальным расстоянием {min_distance} м")
    
    # Генерируем возможные позиции с гарантированным минимальным расстоянием
    def generate_valid_positions():
        """Генерирует валидные позиции с гарантированным минимальным расстоянием"""
        # Создаем сетку возможных позиций с шагом min_distance
        possible_positions = []
        current_pos = min_distance / 2  # начинаем с отступом от начала
        
        while current_pos <= total_length - min_distance / 2:
            possible_positions.append(current_pos)
            current_pos += min_distance
        
        # Если позиций достаточно, выбираем случайные
        if len(possible_positions) >= num_insertions:
            return random.sample(possible_positions, num_insertions)
        else:
            # Если позиций недостаточно, используем более агрессивный подход
            return generate_positions_force()
    
    def generate_positions_force():
        """Альтернативный метод генерации позиций, когда обычный не работает"""
        positions = []
        attempts = 0
        max_attempts = 10000
        
        while len(positions) < num_insertions and attempts < max_attempts:
            attempts += 1
            
            # Генерируем новую позицию
            new_pos = random.uniform(0, total_length)
            
            # Проверяем расстояние до всех существующих позиций
            valid = True
            for pos in positions:
                if abs(new_pos - pos) < min_distance:
                    valid = False
                    break
            
            if valid:
                positions.append(new_pos)
        
        return positions
    
    def generate_positions_grid():
        """Метод сетки для гарантированного нахождения позиций"""
        # Разбиваем трубу на секции длиной min_distance
        num_sections = int(total_length / min_distance)
        
        if num_sections < num_insertions:
            # Если секций меньше чем врезок, используем равномерное распределение
            step = total_length / (num_insertions + 1)
            positions = [step * (i + 1) for i in range(num_insertions)]
            return positions
        
        # Создаем сетку возможных позиций
        grid_positions = [min_distance * (i + 0.5) for i in range(num_sections)]
        
        # Выбираем случайные позиции из сетки
        if len(grid_positions) >= num_insertions:
            return random.sample(grid_positions, num_insertions)
        else:
            return generate_positions_force()
    
    # Пробуем разные методы генерации позиций
    positions = generate_positions_grid()
    
    # Если не удалось получить достаточно позиций, используем принудительный метод
    if len(positions) < num_insertions:
        # Сортируем и выбираем первые num_insertions позиций с минимальным расстоянием
        positions.sort()
        
        # Вычисляем равномерное распределение
        step = total_length / (num_insertions + 1)
        positions = [step * (i + 1) for i in range(num_insertions)]
    
    # Сортируем позиции для удобства
    positions.sort()
    
    insertions = []
    attempts = 0
    max_attempts = 1000
    
    while len(insertions) < num_insertions and attempts < max_attempts:
        attempts += 1
        
        # Случайным образом выбираем позицию на трубе
        random_position = random.uniform(0, total_length)
        
        # Находим сегмент и позицию внутри сегмента
        current_length = 0
        segment_index = -1
        position_in_segment = 0
        
        for i, seg_length in enumerate(segment_lengths):
            if random_position <= current_length + seg_length:
                segment_index = i
                position_in_segment = random_position - current_length
                break
            current_length += seg_length
        
        if segment_index == -1:
            continue
        
        # Получаем точку на трубе
        insertion_point = point_on_segment(segments[segment_index], position_in_segment)
        
        # Проверяем минимальное расстояние до существующих врезок
        too_close = False
        for existing_insertion in insertions:
            distance = calculate_distance(insertion_point, existing_insertion['point'])
            if distance < min_distance:
                too_close = True
                break
        
        if too_close:
            continue
        
        main_angle = calculate_angle(segments[segment_index])
        
        # Случайно выбираем направление ответвления (влево или вправо на 90 градусов)
        branch_direction = random.choice([-1, 1])  # -1 для левого, 1 для правого ответвления
        branch_angle = main_angle + branch_direction * math.pi / 2
        
        branch_end_x = insertion_point[0] + branch_length * math.cos(branch_angle)
        branch_end_y = insertion_point[1] + branch_length * math.sin(branch_angle)
        
        center_x = (insertion_point[0] + branch_end_x) / 2
        center_y = (insertion_point[1] + branch_end_y) / 2
        # Создаем информацию о врезке
        insertion = {
            'point': insertion_point,
            'center_point': (center_x, center_y),
            'branch_angle': branch_angle,
            'branch_length': branch_length
        }
        
        insertions.append(insertion)
    
    if len(insertions) < num_insertions:
        print(f"Предупреждение: удалось разместить только {len(insertions)} из {num_insertions} врезок")
    
    return insertions


def generate_coords():
    global MAIN_LINE_COUNT, START_COORDS, SUM_LENGTH, LINE_COORDS, LINE_COORDS_UNSPLIT

    if MAIN_LINE_COUNT == 0:
        SUM_LENGTH -= random.uniform(0, 5)
        line_length = round(SUM_LENGTH, 2)
        line_angle = random.randint(0, 90)
    else: 
        line_length = round(SUM_LENGTH, 2)
        line_angle = random.randint(-30, 30)


    line_angle = round(line_angle * math.pi / 180, 5)
    end_coords = [round(line_length * math.cos(line_angle) + START_COORDS[0], 5), round(line_length * math.sin(line_angle) + START_COORDS[1], 5)]

    cx, cy = (START_COORDS[0] + end_coords[0]) / 2, (START_COORDS[1] + end_coords[1]) / 2

    LINE_COORDS.append([START_COORDS, end_coords])
    LINE_COORDS_UNSPLIT.append(START_COORDS)
    LINE_COORDS_UNSPLIT.append(end_coords)    
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
         

def main():
    global SUM_LENGTH
    SUM_LENGTH = random.randint(5, 10)
    for i in range(2):
        generate_coords()
    insts = generate_pipeline_insertions(LINE_COORDS_UNSPLIT)
    for inst in insts:
        x, y = inst['center_point']
        z = HEIGHT_ABOVE_FLOOR
        roll, pitch = 0, 0
        yaw = inst['branch_angle']
        length, width, thick = inst['branch_length'], CUT_WIDTH, 0.001

        place_line((x, y, z, roll, pitch, yaw), (length, width, thick))


if __name__ == '__main__':
    main()
