import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
from robomaster import robot
import copy
import threading
import time
from multi_robomaster import multi_robot
import sys
from collections import deque

first_stop=False
second_stop=False
third_stop=False

first_marker=None
second_marker=None
third_marker=None

first_cur_coords=(0,14)
second_cur_coords=(16,0)
third_cur_coords=(25,2)


first_coords=(0,14)
second_coords=(16,0)
third_coords=(25,2)

def find_path(matrix, start, end):
    rows, cols = len(matrix), len(matrix[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Вверх, вниз, влево, вправо
    queue = deque([(start, [start])])
    visited = set()
    visited.add(start)

    while queue:
        (current, path) = queue.popleft()
        if current == end:
            return path

        for direction in directions:
            next_row, next_col = current[0] + direction[0], current[1] + direction[1]
            if 0 <= next_row < rows and 0 <= next_col < cols and matrix[next_row][next_col] == 0:
                next_point = (next_row, next_col)
                if next_point not in visited:
                    visited.add(next_point)
                    queue.append((next_point, path + [next_point]))

    return None  # Если пути нет

def rotate_gimbal(robot_group,num):
    ep_robot = robot_group.get_robot(num)
    ep_gimbal = ep_robot.gimbal
    ep_gimbal.moveto(pitch=10,yaw=0,pitch_speed=10, yaw_speed=30).wait_for_completed()
    while True:
        if num==0 and first_stop:
            break
        if num==1 and second_stop:
            break
        if num==2 and third_stop:
            break    
        ep_gimbal.moveto(pitch=10,yaw=190,pitch_speed=10, yaw_speed=20).wait_for_completed()
        ep_gimbal.moveto(pitch=10,yaw=-190,pitch_speed=10, yaw_speed=20).wait_for_completed()
            

def get_stream(robot_group,num):
    global first_stop, second_stop, third_stop, first_marker,second_marker, third_marker, first_cur_coords,second_cur_coords, third_cur_coords
    ep_robot = robot_group.get_robot(num)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)
    while (True):
        frame = ep_camera.read_cv2_image()
        corners, ids, rejectedCandidates = detector.detectMarkers(frame)
        if len(corners) > 0:
            for i in range(len(ids)):
                c = corners[i][0]
                if ids[i]>=0 and ids[i]<5:
                    robot_coords=None
                    if num==0:
                        robot_coords=first_cur_coords
                    elif num==1:
                        robot_coords=second_cur_coords
                    elif num==2:
                        robot_coords=third_cur_coords
                    if ids[i]==num and num==0:
                        first_stop=True
                        first_marker=robot_coords
                    if ids[i]==num and num==1:
                        second_stop=True
                        second_marker=robot_coords
                    if ids[i]==num and num==2:
                        third_stop=True
                        third_marker=robot_coords 
                    if ids[i]!=num and ids[i]==0:
                        first_marker=robot_coords
                        print(f'Найден первый маркер, его координаты -  {first_marker}')
                    if ids[i]!=num and ids[i]==1:
                        second_marker=robot_coords
                        print(f'Найден второй маркер, его координаты -  {second_marker}')
                    if ids[i]!=num and ids[i]==2:
                        third_marker=robot_coords
                        print(f'Найден третий маркер, его координаты -  {third_marker}')
                    
                       
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        frame_markers=cv2.resize(frame_markers,(640,360))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            #sys.exit()
        cv2.imshow(f'robot {num}', frame_markers)
    cv2.destroyAllWindows()
    
def dfs(matrix, i, j, visited, path):
    if i < 0 or j < 0 or i >= len(matrix) or j >= len(matrix[0]) or matrix[i][j] == 1:
        return
    if visited[i][j]:
        return
    path.append((i, j))
    visited[i][j] = True
    dfs(matrix, i+1, j, visited, path)
    if path[-1] != (i, j):  # Check if the last element in the path is not the current position
        path.append((i, j))  # Add the current position to the path after moving
    dfs(matrix, i-1, j, visited, path)
    if path[-1] != (i, j):  # Check if the last element in the path is not the current position
        path.append((i, j))  # Add the current position to the path after moving
    dfs(matrix, i, j+1, visited, path)
    if path[-1] != (i, j):  # Check if the last element in the path is not the current position
        path.append((i, j))  # Add the current position to the path after moving
    dfs(matrix, i, j-1, visited, path)
    if path[-1] != (i, j):  # Check if the last element in the path is not the current position
        path.append((i, j))  # Add the current position to the path after moving

def traverse_matrix(matrix, start):
    visited = [[False]*len(matrix[0]) for _ in range(len(matrix))]
    path = []
    i, j = start
    if matrix[i][j] == 0 and not visited[i][j]:
        dfs(matrix, i, j, visited, path)
    return path
"""
matrix = [
[1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1],
[1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
[1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
[1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
[1,0,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0],
[1,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0],
[1,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
[1,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0],
[1,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0],
[1,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
[0,0,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0],
[1,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0],
[1,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,1,0,0,0,0,0,1,1,1,1,0,0,0,0,0,1],
[1,1,0,0,0,0,0,1,1,1,1,0,0,0,0,0,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,1,0,1,1,1,0,0,0,0,0,1,1,1,1,1,1],
[1,1,0,1,1,1,0,0,0,0,0,1,1,1,1,1,1],
]
"""
matrix = [
[1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1],
[1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1],
[1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1],
[1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[0,0,0,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,1,1,0,0,0,0,0,0,0,0,0,1,1,0,0,1],
[1,1,1,0,1,0,1,1,1,1,1,0,1,0,0,1,1],
[1,1,1,0,1,0,1,1,1,1,1,0,0,0,1,1,1],
[1,1,0,0,1,0,0,1,1,1,0,0,1,1,1,1,1],
[1,1,0,1,1,1,0,0,0,0,0,1,1,1,1,1,1],
[1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
]

def group_task(robot_group,num,start):  
    global first_marker, second_marker, third_marker, first_stop,second_stop, third_stop, first_cur_coords, second_cur_coords,third_cur_coords
    ep_robot = robot_group.get_robot(num)
    ep_chassis=ep_robot.chassis
    path = traverse_matrix(matrix, start)
    newx=0
    newy=0
    coords=start[:]
    print(f'путь для робота {num}')
    print(path)
    thr=threading.Thread(target=lambda:get_stream(robot_group,num))
    thr.start()
    thr2=threading.Thread(target=lambda:rotate_gimbal(robot_group,num))
    thr2.start()
    for p in path:
        if first_marker:
            if first_stop and num==0:
                print('робот 0 остановлен')
                break
            elif num==0:
                new_path=find_path(matrix,first_cur_coords,first_marker)
                print(f'новый маршрут для робота {num}')
                print(new_path)
                for np in new_path:
                    
                    if coords[0]==np[0]:
                        newx=0
                    else:
                        newx=(p[0]-coords[0])/1.99
                    if coords[1]==np[1]:
                        newy=0
                    else:
                        newy=(p[1]-coords[1])/-1.99
                    #ep_chassis.move(x=newx,y=newy,z=0,xy_speed=1.5).wait_for_completed()
                    first_cur_coords=np
                    coords=np 
                    print(f'робот номер {num} находится в точке {coords}')
                first_stop=True
                break
        if second_marker:
            if second_stop and num==1:
                print('робот 1 остановлен')
                break
            elif num==1:
                new_path=find_path(matrix,second_cur_coords,second_marker)
                print(f'новый маршрут для робота {num}')
                print(new_path)
                for np in new_path:
                    if coords[0]==np[0]:
                        newx=0
                    else:
                        newx=(p[0]-coords[0])/1.99
                    if coords[1]==np[1]:
                        newy=0
                    else:
                        newy=(p[1]-coords[1])/-1.99
                    #ep_chassis.move(x=newx,y=newy,z=0,xy_speed=1.5).wait_for_completed()
                    second_cur_coords=np
                    coords=np   
                    print(f'робот номер {num} находится в точке {coords}')
                second_stop=True
                break
        if third_marker:
            if third_stop and num==2:
                print('робот 2 остановлен')
                break
            elif num==2:
                new_path=find_path(matrix,third_cur_coords,third_marker)
                print(f'новый маршрут для робота {num}')
                print(new_path)
                for np in new_path:
                    if coords[0]==np[0]:
                        newx=0
                    else:
                        newx=(p[0]-coords[0])/1.99
                    if coords[1]==np[1]:
                        newy=0
                    else:
                        newy=(p[1]-coords[1])/-1.99
                    #ep_chassis.move(x=newx,y=newy,z=0,xy_speed=1.5).wait_for_completed()
                    third_cur_coords=np
                    coords=np 
                    print(f'робот номер {num} находится в точке {coords}')
                third_stop=True
                break
        time.sleep(1)
        print(f'робот номер {num} находится в точке {coords}')
        if coords[0]==p[0]:
            newx=0
        else:
            newx=(p[0]-coords[0])/1.99
        if coords[1]==p[1]:
            newy=0
        else:
            newy=(p[1]-coords[1])/-1.99
        #ep_chassis.move(x=newx,y=newy,z=0,xy_speed=1.5).wait_for_completed()
        if num==0:
            first_cur_coords=p
        elif num==1:
            second_cur_coords=p
        elif num==2:
            third_cur_coords=p
        coords=p
# Destroy all the windows
    thr.join()
    thr2.join()
    cv2.destroyAllWindows()
def start_first(start):
    multi_robots.run([robot_group0, lambda x: group_task(robot_group0,0,start)])
def start_second(start):
    multi_robots.run([robot_group1, lambda x: group_task(robot_group1,1,start)])
def start_third(start):
    multi_robots.run([robot_group2, lambda x: group_task(robot_group2,2,start)])


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
robots_sn_list = ['3JKCHCH001043S','3JKCHCH001049C','3JKCHCH00103XS']#'3JKCHCH001043S', '3JKCHCH001043B'
multi_robots = multi_robot.MultiEP()
multi_robots.initialize()
number = multi_robots.number_id_by_sn([0, robots_sn_list[0]], [1, robots_sn_list[1]],[2, robots_sn_list[2]])
print("The number of robot is: {0}".format(number))
robot_group0 = multi_robots.build_group([0])
robot_group1 = multi_robots.build_group([1])
robot_group2 = multi_robots.build_group([2])
#multi_robots.run([robot_group0, lambda x: group_task(robot_group0,0)])
#multi_robots.run([robot_group1, lambda x: group_task(robot_group1,1)])
t1 = threading.Thread(target=lambda:start_first(first_coords))
t2 = threading.Thread(target=lambda:start_second(second_coords))
t3 = threading.Thread(target=lambda:start_third(third_coords))
t1.start()
t2.start()
t3.start()
t1.join()
t2.join()
t3.join()
print("Game over")
multi_robots.close()

