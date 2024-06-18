import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
#import pandas as pd

vid = cv2.VideoCapture(0)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
while (True):

    ret, frame = vid.read()
    #cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    corners, ids, rejectedCandidates = detector.detectMarkers(frame)
    if len(corners) > 0:
            for i in range(len(ids)):
                c = corners[i][0]
                print(f"Робот номер Нашел маркер {ids[i]} corners:")
                #for j in range(4):
                    #print(f"  Corner {j+1}: ({int(c[j][0])}, {int(c[j][1])})")
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    cv2.imshow('frame', frame_markers)
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()