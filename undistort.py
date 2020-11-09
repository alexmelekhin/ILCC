import cv2
import argparse
import os
from os import path as osp
import numpy as np


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-img-fld', '--images-folder', type=str, required=True)
    parser.add_argument('-mtx', type=float, nargs=9, default=[699.9550170898438, 0.0, 632.7949829101562, 0.0, 700.0, 336.2749938964844, 0.0, 0.0, 1.0])
    parser.add_argument('-dist', type=float, nargs=5, default=[-0.17372600734233856, 0.02697340026497841, -1.464870030831733e-10, 8.846759737934917e-05, 5.1472401537466794e-05])
    parser.add_argument('-out-fld', '--out-folder', type=str, required=True)
    return parser


def undistort(image, mtx, dist):
    h, w = image.shape[:2]
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)
    print list(new_mtx.reshape(1, 9)[0])
    undistorted = cv2.undistort(image, mtx, dist, None, new_mtx)
    x, y, w, h = roi
    undistorted = undistorted[y:y + h, x:x + w]
    return undistorted


def undistort_folder(images_folder, mtx, dist, out_folder):
    image_files = os.listdir(images_folder)
    for image_file in image_files:
        image = cv2.imread(osp.join(images_folder, image_file))
        undistorted = undistort(image, mtx, dist)
        cv2.imwrite(osp.join(out_folder, image_file), undistorted)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    mtx = np.array(args.mtx).reshape(3, 3)
    dist = np.array(args.dist)
    undistort_folder(args.images_folder, mtx, dist, args.out_folder)
