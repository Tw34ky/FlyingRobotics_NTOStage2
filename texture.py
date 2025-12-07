import cv2
import numpy as np
import os
import shutil

file_name_1 = 'square_dashed.png'
file_name_2 = 'square_solid.png'
cv2.imwrite(file_name_1, np.array([[[0, 255, 0]]]))
cv2.imwrite(file_name_2, np.array([[[0, 255, 0]]]))

src = f'{os.getcwd()}/{file_name_1}'

dest = '/home/clover/catkin_ws/src/clover/clover_simulation/models/square_line/materials/textures/'
if os.path.isfile(f'{dest}/{file_name_1}'):
    os.remove(f'{dest}/{file_name_1}')
shutil.move(src, dest)


src = f'{os.getcwd()}/{file_name_2}'

dest = '/home/clover/catkin_ws/src/clover/clover_simulation/models/square_line/materials/textures/'
if os.path.isfile(f'{dest}/{file_name_2}'):
    os.remove(f'{dest}/{file_name_2}')
shutil.move(src, dest)