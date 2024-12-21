# SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import cv2
import rclpy
import math
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

class ArucoMarkerDetector():
    def __init__(self, width, height):
        print("Initialized ArucoMarkerDetector")
        self.aruco = cv2.aruco
        self.dictionary = cv2.aruco.Dictionary_get(self.aruco.DICT_4X4_50)

        self.width = width
        self.height = height
        self.angle_of_view = 55.0
        self.k = 0.0
        self.tolerance_error = 30.0
        self.marker_length = 0.085 # actual
        #self.marker_length = 0.1 # simulator
        pkg_dir = get_package_share_directory('aruco_marker_detector_py')
        self.mtx = np.load(os.path.join(pkg_dir, 'mtx.npy'))
        self.dist = np.load(os.path.join(pkg_dir, 'dist.npy'))
        #self.mtx = np.load(os.path.join(pkg_dir, 'mtx_sim.npy'))
        #self.dist = np.load(os.path.join(pkg_dir, 'dist_sim.npy'))
        print(f'mtx = {self.mtx}')
        print(f'dist = {self.dist}')

    def detect_marker(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.aruco.detectMarkers(gray, self.dictionary)
        rtn_ids = []
        XYZ = []
        RPY = []
        if len(corners) != 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.mtx, self.dist)
            for mid, rvec, tvec in zip(ids, rvecs, tvecs):
                #print(f'mtx: {self.mtx}') 
                #print(f'dist: {self.dist}')
                # print(rvec[0][0], tvec)a
                # print(len(rvec), len(tvec))
                R = cv2.Rodrigues(rvec[0])[0]  # 回転ベクトル -> 回転行列
                R_T = R.T
                T = tvec.T
                xyz = np.dot(R_T, - T).squeeze()
                XYZ.append(xyz)
                #print(f'(x, y, z)={xyz}')
                rpy = np.deg2rad(cv2.RQDecomp3x3(R_T)[0])
                RPY.append(rpy)
                # print(f'(r, p, y)={rpy}')
            # # # for mid in ids:
                rtn_ids.append(mid[0])
        drew_marker = self.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        centers = [(int((corner[0][0][0] + corner[0][2][0]) / 2), 
                    int((corner[0][0][1] + corner[0][2][1]) / 2)) for corner in corners]
        # print(ids, XYZ, RPY)
        return centers, rtn_ids, drew_marker, XYZ, RPY

    def position_side(self, center):
        if(self.width/2-center[0] < -self.tolerance_error):
            return -1
        elif(self.width/2-center[0] > self.tolerance_error):
            return 1
        else:
            return 0
    
    def rtn_id_index(self, ids, detected_id):
        try:
            id_index = ids.index(detected_id)
            return id_index

        except: 
            return None

