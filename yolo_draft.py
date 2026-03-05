import rospy
import cv2
import numpy as np

# from ultralytics import YOLO # ЭТО НУЖНО ПОСТАВИТЬ БУДЕТ

rospy.init_node('flight')

from sensor_msgs.msg import Image, PointCloud
from cv_bridge import CvBridge


bridge = CvBridge()


model = None
"""
model = None # ЭТО ДОЛЖНА БЫТЬ НАТРЕНИРОВАННАЯ МОДЕЛЬ YOLO 



"""


def run_prediction(model, source):
    """
    model - YOLO ultralytics
    source (str | Path | int | PIL.Image | np.ndarray | torch.Tensor | list | tuple): The source of the image(s)
                to make predictions on. Accepts various types including file paths, URLs, PIL images, numpy arrays, and
                torch tensors. - это взято из документации yolo @ https://github.com/ultralytics/ultralytics/blob/main/ultralytics/engine/model.py
    
    """
    prediction = model.predict(source)

    """

    Attributes объекта prediction:
        orig_img (np.ndarray): The original image as a numpy array.
        orig_shape (tuple[int, int]): Original image shape in (height, width) format.
        boxes (Boxes | None): Detected bounding boxes.
        masks (Masks | None): Segmentation masks.
        probs (Probs | None): Classification probabilities.
        keypoints (Keypoints | None): Detected keypoints.
        obb (OBB | None): Oriented bounding boxes.
        speed (dict): Dictionary containing inference speed information.
        names (dict): Dictionary mapping class indices to class names.
        path (str): Path to the input image file.
        save_dir (str | None): Directory to save results.

    @ https://github.com/ultralytics/ultralytics/blob/main/ultralytics/engine/results.py#L176
    """

    return prediction


while not rospy.is_shutdown():
    
    rospy.sleep(1)
    data = rospy.wait_for_message('/main_camera/image_raw', Image)
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    img = cv2.CvtColor(img, cv2.COLOR_BGR2RGB)

    img_prediction = run_prediction(model, img)
    # пока что этого хватит, дальше будем работать уже по факту необходимости
    

