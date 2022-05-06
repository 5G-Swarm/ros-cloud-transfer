import pygame
import numpy as np
import socket
import threading
import time
import evdev
from evdev import ecodes, InputDevice

HOST_ADDRESS = '127.0.0.1'
BLACK = (0, 0, 0)
GREY = (192, 192, 192)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
WHITE = (255, 255, 255)
WINDOW_WIDTH = 1920
WINDOW_HEIGHT = 1080
ROBOT_SIZE = 20
BUTTON_WIDTH = 300
BUTTON_HEIGHT = 100
BUTTON_LIGHT = (170, 170, 170)
BUTTON_DARK = (100, 100, 100)
BUTTON_GOAL_X = 50
BUTTON_GOAL_Y = 50
BUTTON_BAIDU_X = 50
BUTTON_BAIDU_Y = 200
BUTTON_SATELLITE_X = 50
BUTTON_SATELLITE_Y = 350
BUTTON_JOYSTICK_X = 50
BUTTON_JOYSTICK_Y = 500


def setup_joystick():
    try:
        device = evdev.list_devices()[0]
        evtdev = InputDevice(device)
        val = 25000 #[0,65535]
        evtdev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, val)
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    except:
        print('JOYSTICK NOT CONNECTED!!!')

def parse_vehicle_wheel(joystick, clock):
    keys = pygame.key.get_pressed()
    milliseconds = clock.get_time()

    throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
    steer_increment = 5e-4 * milliseconds
    if keys[K_LEFT] or keys[K_a]:
        steer_cache -= steer_increment
    elif keys[K_RIGHT] or keys[K_d]:
        steer_cache += steer_increment
    else:
        steer_cache = 0.0
    steer_cache = min(0.7, max(-0.7, steer_cache))
    steer = round(steer_cache, 1)
    brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0

    numAxes = joystick.get_numaxes()
    jsInputs = [float(joystick.get_axis(i)) for i in range(numAxes)]

    # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
    # For the steering, it seems fine as it is
    K1 = 1.0  # 0.55
    steerCmd = K1 * math.tan(1.1 * jsInputs[0])

    K2 = 1.6  # 1.6
    throttleCmd = K2 + (2.05 * math.log10(
        -0.7 * jsInputs[2] + 1.4) - 1.2) / 0.92
    if throttleCmd <= 0:
        throttleCmd = 0
    elif throttleCmd > 1:
        throttleCmd = 1

    brakeCmd = 1.6 + (2.05 * math.log10(
        -0.7 * jsInputs[3] + 1.4) - 1.2) / 0.92
    if brakeCmd <= 0:
        brakeCmd = 0
    elif brakeCmd > 1:
        brakeCmd = 1

    steer = steerCmd
    brake = brakeCmd
    throttle = throttleCmd

    return steer, throttle, brake

def drawJoystick(steer, throttle, brake):
    # font settings
    FONT = pygame.font.SysFont('Corbel', 50)
    text = FONT.render('steer {:.2f}'.format(steer), True, BLUE)
    SCREEN.blit(text, (BUTTON_JOYSTICK_X+325, BUTTON_JOYSTICK_Y))
    text = FONT.render('throttle {:.2f}'.format(throttle), True, BLUE)
    SCREEN.blit(text, (BUTTON_JOYSTICK_X+325, BUTTON_JOYSTICK_Y+35))
    text = FONT.render('brake {:.2f}'.format(brake), True, BLUE)
    SCREEN.blit(text, (BUTTON_JOYSTICK_X+325, BUTTON_JOYSTICK_Y+70))

def drawMaps(SCREEN, DISPLAY_MAP, map_offset):
    WINDOW_WIDTH, WINDOW_HEIGHT = pygame.display.get_surface().get_size()
    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    map_pos = np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2]) + map_offset
    SCREEN.blit(DISPLAY_MAP, map_pos)

def drawFixedGoal(SCREEN, fixed_goal, map_offset):
    for idx, goal in enumerate(fixed_goal):
        FONT = pygame.font.SysFont('Corbel', 100)
        text = FONT.render('{}'.format(idx+1), True, GREEN)
        SCREEN.blit(text, goal + map_offset + np.array([-25, -25]))

def drawGoal(SCREEN, robot_goal, map_offset):
    if robot_goal is not None:
        # pygame.draw.circle(SCREEN, GREEN, robot_goal + map_offset, ROBOT_SIZE)
        cicle = (robot_goal + map_offset)
        marker_size = 20
        width = 10
        pygame.draw.line(SCREEN, RED, (cicle[0]-marker_size, cicle[1]-marker_size), (cicle[0]+marker_size, cicle[1]+marker_size), width)
        pygame.draw.line(SCREEN, RED, (cicle[0]-marker_size, cicle[1]+marker_size), (cicle[0]+marker_size, cicle[1]-marker_size), width)

def drawRobots(SCREEN, robot_dict, map_offset):
    for robot in robot_dict.items():
        cmd = [0.5]
        pygame.draw.circle(SCREEN, GREEN, robot.pos + map_offset, ROBOT_SIZE)
        pygame.draw.line(SCREEN, BLUE, robot.pos + map_offset, robot.pos + map_offset + min(max(40*cmd[0], 25), 40)*np.array([np.cos(robot.heading+np.pi/2), -np.sin(robot.heading+np.pi/2)]), 5)

def drawBoundingBox(SCREEN, bounding_box, map_offset):
    bounding_box_copy = bounding_box.copy()
    for _, pos in bounding_box_copy.items():
        # x, y = pos + map_offset
        # pygame.draw.rect(SCREEN, BLUE, pygame.Rect(x, y, 60, 100), 10)
        pygame.draw.lines(SCREEN, BLUE, True, pos + map_offset, 10)

def drawPath(SCREEN, path_pos, map_offset):
    if len(path_pos) > 1:
        pygame.draw.lines(SCREEN, RED, False, path_pos + map_offset, 10)

def drawButton(SCREEN, use_baidu_map, use_satellite_map, use_joystick):
    # font settings
    FONT = pygame.font.SysFont('Corbel', 75)

    # get mouse position
    mouse = pygame.mouse.get_pos()

    # button: set goal
    text = FONT.render('Set Goal', True, WHITE)
    if BUTTON_GOAL_X <= mouse[0] <= BUTTON_GOAL_X + BUTTON_WIDTH and BUTTON_GOAL_Y <= mouse[1] <= BUTTON_GOAL_Y + BUTTON_HEIGHT:
        pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_GOAL_X, BUTTON_GOAL_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    else:
        pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_GOAL_X, BUTTON_GOAL_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    SCREEN.blit(text, (BUTTON_GOAL_X+45, BUTTON_GOAL_Y+25))
    # button: baidu map
    text = FONT.render('BAIDU', True, WHITE)
    if (BUTTON_BAIDU_X <= mouse[0] <= BUTTON_BAIDU_X + BUTTON_WIDTH and BUTTON_BAIDU_Y <= mouse[1] <= BUTTON_BAIDU_Y + BUTTON_HEIGHT) or use_baidu_map:
        pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_BAIDU_X, BUTTON_BAIDU_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    else:
        pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_BAIDU_X, BUTTON_BAIDU_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    SCREEN.blit(text, (BUTTON_BAIDU_X+60, BUTTON_BAIDU_Y+25))
    # button: satellite map
    text = FONT.render('SATELLITE', True, WHITE)
    if (BUTTON_SATELLITE_X <= mouse[0] <= BUTTON_SATELLITE_X + BUTTON_WIDTH and BUTTON_SATELLITE_Y <= mouse[1] <= BUTTON_SATELLITE_Y + BUTTON_HEIGHT) or use_satellite_map:
        pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_SATELLITE_X, BUTTON_SATELLITE_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    else:
        pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_SATELLITE_X, BUTTON_SATELLITE_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    SCREEN.blit(text, (BUTTON_SATELLITE_X+10, BUTTON_SATELLITE_Y+25))
    # button: joystick mode
    text = FONT.render('JOYSTICK', True, WHITE)
    if (BUTTON_JOYSTICK_X <= mouse[0] <= BUTTON_JOYSTICK_X + BUTTON_WIDTH and BUTTON_JOYSTICK_Y <= mouse[1] <= BUTTON_JOYSTICK_Y + BUTTON_HEIGHT) or use_joystick:
        pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_JOYSTICK_X, BUTTON_JOYSTICK_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    else:
        pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_JOYSTICK_X, BUTTON_JOYSTICK_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    SCREEN.blit(text, (BUTTON_JOYSTICK_X+20, BUTTON_JOYSTICK_Y+25))

def drawMessageBox(SCREEN, map_offset, robot_clicked, robot_clicked_id, robot_dict, box_clicked, box_clicked_id, bounding_box):
    # font settings
    FONT = pygame.font.SysFont('Corbel', 75)

    # get mouse position
    mouse = pygame.mouse.get_pos()

    if robot_clicked:
        # box
        BOX_X, BOX_Y = robot_dict[robot_clicked_id].pos + map_offset + np.array([25, -150])
        BOX_WIDTH, BOX_HEIGHT = 350, 150
        BOX_COLOR = (255, 255, 255)
        pygame.draw.rect(SCREEN, BOX_COLOR, [BOX_X, BOX_Y, BOX_WIDTH, BOX_HEIGHT])

        # button: view image
        text = FONT.render('View Image', True, WHITE)
        BUTTON_IMAGE_X, BUTTON_IMAGE_Y = robot_dict[robot_clicked_id].pos + map_offset + np.array([50, -125])
        if BUTTON_IMAGE_X <= mouse[0] <= BUTTON_IMAGE_X + BUTTON_WIDTH and BUTTON_IMAGE_Y <= mouse[1] <= BUTTON_IMAGE_Y + BUTTON_HEIGHT:
            pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_IMAGE_X, BUTTON_IMAGE_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        else:
            pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_IMAGE_X, BUTTON_IMAGE_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        SCREEN.blit(text, (BUTTON_IMAGE_X+10, BUTTON_IMAGE_Y+25))

    if box_clicked:
        # box
        box_center = np.mean(bounding_box[box_clicked_id], axis=0)
        BOX_X, BOX_Y = box_center + map_offset + np.array([25, -150])
        BOX_WIDTH, BOX_HEIGHT = 350, 150
        BOX_COLOR = (255, 255, 255)
        pygame.draw.rect(SCREEN, BOX_COLOR, [BOX_X, BOX_Y, BOX_WIDTH, BOX_HEIGHT])

        # button: get id
        text = FONT.render('Get ID', True, WHITE)
        BUTTON_ID_X, BUTTON_ID_Y = box_center + map_offset + np.array([50, -125])
        if BUTTON_ID_X <= mouse[0] <= BUTTON_ID_X + BUTTON_WIDTH and BUTTON_ID_Y <= mouse[1] <= BUTTON_ID_Y + BUTTON_HEIGHT:
            pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_ID_X, BUTTON_ID_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        else:
            pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_ID_X, BUTTON_ID_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        SCREEN.blit(text, (BUTTON_ID_X+75, BUTTON_ID_Y+25))

def drawRectSelections(SCREEN, rect_select, rect_start_pos, rect_end_pos):
    if rect_select and rect_start_pos is not None and rect_end_pos is not None:
        pygame.draw.rect(SCREEN, GREY, [rect_start_pos, np.array(rect_end_pos) - np.array(rect_start_pos)], 5)


class Receiver(object):
    def __init__(self):
        self.path_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.path_sock.settimeout(1.0)
        self.path_sock.bind((HOST_ADDRESS, 23333))
        self.path_thread = threading.Thread(target=self.receive_path)
        self.path_thread.start()
        self.gesture_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gesture_sock.settimeout(1.0)
        self.gesture_sock.bind((HOST_ADDRESS, 23335))
        self.gesture_thread = threading.Thread(target=self.receive_gesture)
        self.gesture_thread.start()
        self.timeout = False

    def receive_path(self):
        global path_pos, new_path_pos
        while True:
            try:
                data, _ = self.path_sock.recvfrom(4096)
                data = data.decode("utf-8").split(';')
                MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
                offset = np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2])
                path_pos = np.array([np.array([float(pos.split(',')[0]), float(pos.split(',')[1])]) + offset
                            for pos in data if pos != ''])
                new_path_pos = np.array([np.array([float(pos.split(',')[0]), float(pos.split(',')[1])])
                            for pos in data if pos != ''])
                # print(path_pos, len(path_pos))
                self.timeout = False
            except socket.timeout:
                self.timeout = True
            time.sleep(0.01)

    def receive_gesture(self):
        while True:
            try:
                data, _ = self.gesture_sock.recvfrom(4096)
                gesture = data.decode("utf-8")
                # print(gesture)
                global robot_goal
                if gesture == 'Number 1':
                    robot_goal = fixed_goal[0]
                elif gesture == 'Number 2':
                    robot_goal = fixed_goal[1]
                elif gesture == 'Number 3':
                    robot_goal = fixed_goal[2]
                elif gesture == 'Number 4':
                    robot_goal = fixed_goal[3]
                elif gesture == 'Number 5':
                    robot_goal = fixed_goal[4]
                self.timeout = False
            except socket.timeout:
                self.timeout = True
            time.sleep(0.01)

def sendGoal(robot_dict, goal):
    goal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_id = 0
    if robot_id not in robot_dict.keys():
        goal_str = str(goal[0]) + ',' + str(goal[1])
    else:
        MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
        offset = np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2])
        pos = robot_dict[robot_id].pos - offset
        goal_str = str(goal[0]) + ',' + str(goal[1]) + ',' + str(pos[0]) + ',' + str(pos[1])
    goal_sock.sendto(bytes(goal_str, 'ascii'), (HOST_ADDRESS, 23334))

class Robot():
    def __init__(self, id, pos=None, heading=None, cmd=None, img=None):
        self.id = id
        self.pos = pos
        self.heading = heading
        self.cmd = cmd
        self.img = img

    def update_pos(self, pos):
        self.pos = pos

    def update_heading(self, heading):
        self.heading = heading

    def update_cmd(self, cmd):
        self.cmd = cmd

    def update_img(self, img):
        self.img = img
