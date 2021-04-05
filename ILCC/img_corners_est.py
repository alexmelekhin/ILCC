import numpy as np
import cv2
import os
from ast import literal_eval as make_tuple
import config
import shutil
import sys
from config import dataset

print np
params = config.default_params()

show_corners = False
skip_ids = (28, 29,)


# corner detection from one image
def get_corner_coords(imagefilename, backend=params['backend'], size=make_tuple(params['pattern_size']),
                      save_corners=params['output_img_with_dectected_corners']):
    if backend == "matlab":
        try:
            import matlab.engine
            print "Matlab is used as backend for detecting corners"
        except ImportError:
            print "matlab.engine can not be found!"
            print "To use detectCheckerboardPoints function of matlab in python, matlab.engine for python should be installed!"
            sys.exit(0)

        eng = matlab.engine.start_matlab()
        imagePoints, boardSize, imagesUsed = eng.detectCheckerboardPoints(imagefilename, nargout=3)
        print boardSize, imagesUsed
        if not imagesUsed:
            print "Corners can not be detected!"
            return None

        np_imagePoints = np.array(imagePoints)
        if save_corners or show_corners:
            img = cv2.imread(imagefilename)
            size = tuple((np.array(boardSize).astype(np.int32) - 1).flatten())
            cv2.drawChessboardCorners(img, size, np_imagePoints.astype(np.float32), 1)
            if save_corners:
                save_imagefilename = os.path.join(params['base_dir'], "output")+"/img_corners/" + \
                                     (imagefilename.split("/")[-1]).split(".")[
                                         0] + "_detected_corners" + "." + params['image_format']
                cv2.imwrite(save_imagefilename, img)
                print "Image with detected_corners is saved in " + save_imagefilename
            if show_corners:
                cv2.imshow("image with detected corners", img)
                while True:
                    k = cv2.waitKey(1)
                    if k == 27:
                        cv2.destroyAllWindows()
                        break
                    if cv2.getWindowProperty("image with detected corners", cv2.WND_PROP_VISIBLE) < 1:
                        break

        return np_imagePoints

    elif backend == "opencv":
        print "OpenCV " + str(cv2.__version__) + " is used as backend for detecting corners"
        img = cv2.imread(imagefilename)
        size=(size[0]-1,size[1]-1)
        print img.shape

        ret, corners = cv2.findChessboardCorners(img, size,
                                                 flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
        #flags=cv2.cv.CV_CALIB_CB_ADAPTIVE_THRESH + cv2.cv.CV_CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        corners_reshaped=corners.reshape((size[1],size[0],2))
        corners_reshaped=np.flip(corners_reshaped,1)
        corners=corners_reshaped.reshape((size[0]*size[1],1,2))
        if not ret:
            print "Corners can not be detected!"
            return None

        cv2.drawChessboardCorners(img, size, corners, ret)
        if show_corners:
            cv2.namedWindow("img", cv2.WINDOW_NORMAL)
            cv2.imshow('img', img)
            while True:
                k = cv2.waitKey(1)
                if k == 27:
                    cv2.destroyAllWindows()
                    break
                if cv2.getWindowProperty('img', cv2.WND_PROP_VISIBLE) < 1:
                    break
        if save_corners:
            save_imagefilename = "output/img_corners/" + (imagefilename.split("/")[-1]).split(".")[
                0] + "_detected_corners" + "." + params['image_format']
            cv2.imwrite(save_imagefilename, img)
        return corners

    else:
        AssertionError("Please input the right backend for corner detection")


#
def detect_img_corners():
    ls = np.arange(1,params['poses_num']+1).tolist()
    img_corner_path = os.path.join(params['base_dir'], "output/img_corners/")
    if os.path.isdir(img_corner_path):
        shutil.rmtree(img_corner_path)
    os.makedirs(img_corner_path)
    for i in ls:
        if i in skip_ids:
            continue
        try:
            imagefilename = os.path.join(params['base_dir'],
                                         "img", str(i).zfill(params['file_name_digits']) + "." + params['image_format'])
            print imagefilename
            corner_points = get_corner_coords(imagefilename)

            # print corner_points
            save_points_filename = img_corner_path + str(i).zfill(
                params['file_name_digits']) + "_img_corners" + ".txt"
            np.savetxt(save_points_filename, np.squeeze(corner_points), delimiter=",")
        except:
            raise RuntimeError()


if __name__ == '__main__':
    detect_img_corners()
