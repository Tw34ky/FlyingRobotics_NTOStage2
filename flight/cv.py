import cv2
import numpy as np
import math


def find_optimal_parallel_lines(lines, binary_img):
    
    if lines is None:
        return None, None, 0

    # Filter and group similar lines
    checked = [lines[0]]
    for i in lines:
        for j in checked:
            deg = abs(i[0][1] - j[0][1])
            if min(deg, math.pi - deg) < 0.17 and abs(abs(i[0][0]) - abs(j[0][0])) < 20:
                break
        else:
            checked.append(i)

    # Find all parallel line pairs and calculate their area/distance ratio
    best_ratio = 0
    best_pair = None
    best_area = 0

    for i in range(len(checked)):
        for j in range(i + 1, len(checked)):
            line1 = checked[i]
            line2 = checked[j]

            rho1, theta1 = line1[0]
            rho2, theta2 = line2[0]

            # Check if lines are parallel (similar theta)
            deg_diff = min(abs(theta1 - theta2), math.pi - abs(theta1 - theta2))
            if deg_diff < 0.30:  # Same parallelism threshold
                # Calculate distance between parallel lines
                distance = abs(rho1 - rho2)

                if distance > 10:  # Minimum distance threshold to avoid very close lines
                    # Calculate white area between the two lines
                    area = calculate_area_between_lines(binary_img, line1, line2)

                    # Calculate ratio
                    if distance > 0:
                        ratio = area / distance

                        if ratio > best_ratio:
                            best_ratio = ratio
                            best_pair = (line1, line2)
                            best_area = area

    return best_pair, best_area, best_ratio


def get_mask(binary_img, line1, line2):
    height, width = binary_img.shape
    mask = np.zeros((height, width), dtype=np.uint8)

    # Get line parameters
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    points1 = get_line_boundary_points(rho1, theta1, width, height)
    points2 = get_line_boundary_points(rho2, theta2, width, height)

    if points1 is None or points2 is None:
        return 0
    pts = np.array([points1[0], points1[1], points2[1], points2[0]], dtype=np.int32)
    cv2.fillPoly(mask, [pts], 255)
    pts = np.array([points1[1], points1[0], points2[1], points2[0]], dtype=np.int32)
    cv2.fillPoly(mask, [pts], 255)
    return mask


def calculate_area_between_lines(binary_img, line1, line2):
    mask = get_mask(binary_img, line1, line2)
    masked_binary = cv2.bitwise_and(binary_img, binary_img, mask=mask)
    area = np.count_nonzero(masked_binary)
    return area


def get_line_boundary_points(rho, theta, width, height):
    """
    Get two points where the line intersects with image boundaries
    """
    a = np.cos(theta)
    b = np.sin(theta)
    points = []

    if abs(b) > 1e-6:

        y_left = (rho - a * 0) / b
        if 0 <= y_left <= height:
            points.append((0, int(y_left)))

        y_right = (rho - a * width) / b
        if 0 <= y_right <= height:
            points.append((width, int(y_right)))

    if abs(a) > 1e-6:
        x_top = (rho - b * 0) / a
        if 0 <= x_top <= width:
            points.append((int(x_top), 0))

        x_bottom = (rho - b * height) / a
        if 0 <= x_bottom <= width:
            points.append((int(x_bottom), height))

    points = list(set(points))
    if len(points) >= 2:
        return points[:2]
    return None


def process_image_with_optimal_lines(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    binary_img = cv2.inRange(hsv, (0, 1, 1), (7, 255, 255))
    edges = cv2.Canny(binary_img, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 45)  # Example

    if lines is not None:
        best_pair, best_area, best_ratio = find_optimal_parallel_lines(lines, binary_img)


        # Draw all checked lines in blue
        checked = [lines[0]]
        for i in lines:
            for j in checked:
                deg = abs(i[0][1] - j[0][1])
                if min(deg, math.pi - deg) < 0.20 and abs(abs(i[0][0]) - abs(j[0][0])) < 20:
                    break
            else:
                checked.append(i)
        
        mask = cv2.bitwise_and(binary_img, get_mask(binary_img, *best_pair))
        no_main = cv2.bitwise_xor(binary_img, mask)
        contours, _ = cv2.findContours(no_main, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            for contour in contours:
                M = cv2.moments(contour)
                try:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    if abs(cx - 120) < 40:
                        cv2.drawContours(no_main, [contour], 0, (0,0,0), -1)
                        # cv2.imshow("twink radar", no_main)
                        # cv2.waitKey(1000)
                        print("twink obliterated")
                except ZeroDivisionError:
                    pass
        return mask, no_main, best_pair, binary_img
        # return binary_img, best_pair
