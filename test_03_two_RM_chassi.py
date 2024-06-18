import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
from robomaster import robot
import threading
import time
from multi_robomaster import multi_robot
import sys
def get_stream(robot_group,num):
    ep_robot = robot_group.get_robot(num)

    
    ep_camera = ep_robot.camera
    # 显示200帧图传
    ep_camera.start_video_stream(display=False)
    while (True):
        frame = ep_camera.read_cv2_image()
        corners, ids, rejectedCandidates = detector.detectMarkers(frame)
        if len(corners) > 0:
            for i in range(len(ids)):
                c = corners[i][0]
                print(f"ArUco marker {ids[i]} corners:")
                for j in range(4):
                    print(f"  Corner {j+1}: ({int(c[j][0])}, {int(c[j][1])})")
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            #sys.exit()
        cv2.imshow(f'robot {num}', frame_markers)
    cv2.destroyAllWindows()
def group_task(robot_group,num):
    
    ep_robot = robot_group.get_robot(num)
    thr=threading.Thread(target=lambda:get_stream(robot_group,num))
    thr.start()
    while True:
        time.sleep(1)
        print('sleep')
    thr.join()
    
       
        
# Destroy all the windows
def start_first():
    multi_robots.run([robot_group0, lambda x: group_task(robot_group0,0)])
def start_second():
    multi_robots.run([robot_group1, lambda x: group_task(robot_group1,1)])

if __name__ == '__main__':
    #get robot sn by run the exmaples of /15_multi_robot/multi_ep/01_scan_robot_sn.py
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    robots_sn_list = ['3JKCHCH001043S','3JKCHCH001043B']#'3JKCHCH001043S', '3JKCHCH001043B'
    multi_robots = multi_robot.MultiEP()
    multi_robots.initialize()
 
    number = multi_robots.number_id_by_sn([0, robots_sn_list[0]], [1, robots_sn_list[1]])
    print("The number of robot is: {0}".format(number))
    robot_group0 = multi_robots.build_group([0])
    robot_group1 = multi_robots.build_group([1])
    #multi_robots.run([robot_group0, lambda x: group_task(robot_group0,0)])
    #multi_robots.run([robot_group1, lambda x: group_task(robot_group1,1)])
    t1 = threading.Thread(target=start_first)
    t2 = threading.Thread(target=start_second)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    print("Game over")
    multi_robots.close()

