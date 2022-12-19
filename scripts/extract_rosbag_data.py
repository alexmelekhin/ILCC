import rosbag
import argparse
import os
from os import path as osp
from pathlib2 import Path
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs import point_cloud2
import math
import csv


bridge = CvBridge()


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-data-fld', '--data-folder', type=str, required=True, help='rosbag files folder')
    parser.add_argument('-img-topic', '--images-topic', type=str, required=True)
    parser.add_argument('-pcd-topic', '--point-clouds-topic', type=str, required=True)
    parser.add_argument('-out-img-fld', '--out-images-folder', type=str, required=True)
    parser.add_argument('-out-pcd-fld', '--out-point-clouds-folder', type=str, required=True)
    parser.add_argument('-do-undist', '--do-undistort', action='store_true')
    parser.add_argument('-mtx', '--camera-matrix', type=str, help="Required if --do-undistort is set.")
    parser.add_argument('-dist', '--distortion-coefficients', type=str, help="Required if --do-undistort is set.")
    return parser


def do_undistort(image, camera_matrix, distortion_coefficients):
    assert (camera_matrix is not None) and (distortion_coefficients is not None)
    undistorted = cv2.undistort(image, camera_matrix, distortion_coefficients)
    assert image.shape == undistorted.shape
    return undistorted


def extract_rosbag_image(bag, images_topic, undistort=False, camera_matrix=None, distortion_coefficients=None):
    global bridge
    image_msg = next(bag.read_messages(topics=[images_topic]))[1]
    if images_topic.endswith('compressed'):
        image = bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
    else:
        image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
    if undistort:
        image = do_undistort(image, camera_matrix, distortion_coefficients)
    return image


def extract_rosbag_point_cloud(bag, point_clouds_topic):
    point_cloud_msg = next(bag.read_messages(topics=[point_clouds_topic]))[1]
    lidar_points = point_cloud2.read_points(point_cloud_msg, field_names=('x', 'y', 'z', 'intensity', 'ring'),
                                            skip_nans=True)
    point_cloud = list()
    point_cloud.append(["Points_m_XYZ:0", "Points_m_XYZ:1", "Points_m_XYZ:2", "intensity", "laser_id", "azimuth",
                        "distance_m", "adjustedtime", "timestamp"])
    for lidar_point in lidar_points:
        x = lidar_point[0]
        y = lidar_point[1]
        z = lidar_point[2]
        intensity = lidar_point[3]
        laser_id = float(lidar_point[4])
        azimuth = np.rad2deg(math.atan2(x, y))
        if azimuth < 0:
            azimuth += 360
        azimuth = math.floor(azimuth * 100)
        distance = np.linalg.norm([x, y, z])
        point_cloud.append([x, y, z, intensity, laser_id, azimuth, distance, 0., 0.])
    return point_cloud


def extract_rosbag_data(bag, images_topic, point_clouds_topic, undistort=False, camera_matrix=None,
                        distortion_coefficients=None):
    image = extract_rosbag_image(bag, images_topic, undistort=undistort, camera_matrix=camera_matrix,
                                 distortion_coefficients=distortion_coefficients)
    point_cloud = extract_rosbag_point_cloud(bag, point_clouds_topic)
    return image, point_cloud


def extract_rosbag_data_folder(data_folder, images_topic, point_clouds_topic, out_images_folder,
                               out_point_clouds_folder, undistort=False, camera_matrix=None,
                               distortion_coefficients=None):
    file_names = os.listdir(data_folder)
    for file_name in file_names:
        if not file_name.endswith('.bag'):
            continue
        if not file_name[:-4].isdigit():
            continue
        bag = rosbag.Bag(osp.join(data_folder, file_name), 'r')
        image, point_cloud = extract_rosbag_data(bag, images_topic, point_clouds_topic, undistort=undistort,
                                                 camera_matrix=camera_matrix,
                                                 distortion_coefficients=distortion_coefficients)
        bag.close()
        cv2.imwrite(osp.join(out_images_folder, file_name[:-4].zfill(4) + '.png'), image)
        with open(osp.join(out_point_clouds_folder, file_name[:-4].zfill(4) + '.csv'), 'w') as f:
            writer = csv.writer(f, quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
            writer.writerows(point_cloud)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()

    Path(args.out_images_folder).mkdir(exist_ok=True)
    Path(args.out_point_clouds_folder).mkdir(exist_ok=True)

    if args.do_undistort:
        camera_matrix = args.camera_matrix
        camera_matrix = camera_matrix.split()
        assert len(camera_matrix) == 9
        camera_matrix = np.array(list(map(float, camera_matrix))).reshape(3, 3)

        distortion_coefficients = args.distortion_coefficients
        distortion_coefficients = distortion_coefficients.split()
        assert len(distortion_coefficients) == 5
        distortion_coefficients = np.array(list(map(float, distortion_coefficients)))
    else:
        camera_matrix = None
        distortion_coefficients = None

    extract_rosbag_data_folder(args.data_folder, args.images_topic, args.point_clouds_topic, args.out_images_folder,
                               args.out_point_clouds_folder, undistort=args.do_undistort, camera_matrix=camera_matrix,
                               distortion_coefficients=distortion_coefficients)
        
