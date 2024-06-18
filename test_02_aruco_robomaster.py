import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
from robomaster import robot
import time
#import pandas as pd
ep_robot = robot.Robot()
ep_robot.initialize(conn_type='ap')

ep_camera = ep_robot.camera
ep_gimbal = ep_robot.gimbal

ep_camera.start_video_stream(display=False)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
while True:
    frame = ep_camera.read_cv2_image()
    #ret, frame = vid.read()
    #cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    corners, ids, rejectedCandidates = detector.detectMarkers(frame)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    frame_markers=cv2.resize(frame_markers,(640,360))
    cv2.imshow('frame', frame_markers)
vid.release()

cv2.destroyAllWindows()
