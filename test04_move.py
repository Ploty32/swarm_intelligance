import robomaster
from robomaster import robot

# Функция для обработки данных с камеры
def detect_color_card(det_res):
    print(det_res)
    for color_card in det_res:
        # Определение цвета карты
        card_color = color_card['results'][0]['color']
        
        # Определение положения карты
        card_position = color_card['location']
        
        # Определение, куда двигаться в зависимости от положения карты
        if card_position['x'] < 0:
            # Карта находится слева, двигаемся вправо
            robot.chassis.move(x=1, y=0, z=0, xy_speed=0.7).wait_for_completed()
        elif card_position['x'] > 0:
            # Карта находится справа, двигаемся влево
            robot.chassis.move(x=-1, y=0, z=0, xy_speed=0.7).wait_for_completed()
        else:
            # Карта находится по центру, двигаемся прямо
            robot.chassis.move(x=0, y=1, z=0, xy_speed=0.7).wait_for_completed()

# Функция для обхода помещения
def explore_room(robot):
    # Инициализация камеры
    robot.vision.sub_detect_info("detect_color_card")

    # Движение робота
    robot.chassis.move(x=1, y=0, z=0, xy_speed=0.7).wait_for_completed()

    # Остановка подписки на камеру
    robot.vision.unsub_detect_info("detect_color_card")

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta",sn='3JKCHCH001043B')
    explore_room(ep_robot)
    ep_robot.close()