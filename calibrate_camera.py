import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def read_ros_arguments():
    return


def receive_command():
    command = input('command: ')
    if command in ['r']:
        return 'receive_image'
    elif command in ['c']:
        return 'calibrate'
    elif command in ['q']:
        return 'quit'
    elif command in ['']:
        return 'no_command'
    else:
        raise RuntimeError('Unknown command: {}\n'.format(command))


def receive_image():
    msg = rospy.wait_for_message('zed_node/left_raw/image_raw_color', Image)
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    return image


def calibrate(images):
    termination_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_points = np.zeros((6*8, 3), np.float32)
    obj_points[:, :2] = np.mgrid[0:6, 0:8].T.reshape(-1,2)
    images_points = list()
    objects_points = list()
    for image in images:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern_found, img_points = cv2.findChessboardCorners(gray, (6, 8), None)
        if not pattern_found:
            raise RuntimeError('Chessboard pattern could not be found\n')
        img_points = cv2.cornerSubPix(gray, img_points, (11, 11), (-1, -1), termination_criteria)
        drawn = cv2.drawChessboardCorners(image, (6, 8), img_points, pattern_found)
        cv2.imshow('image', drawn)
        cv2.waitKey(0)
        objects_points.append(obj_points)
        images_points.append(img_points)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objects_points, images_points, gray.shape[::-1], None, None)
    print(mtx)
    print(dist)


if __name__ == '__main__':
    rospy.init_node('calibrate_camera')
    bridge = CvBridge()
    images = list()
    while True:
        command = receive_command()
        if command == 'receive_image':
            image = receive_image()
            images.append(image)
        elif command == 'calibrate':
            calibrate(images)
        elif command == 'quit':
            break
        elif command == 'no_command':
            continue
        else:
            raise RuntimeError()
        
        
