from robomaster import robot

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

ep_version = ep_robot.get_version()
print("Robot Version: {0}".format(ep_version))

ep_robot.close()