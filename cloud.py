#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from os import wait
from turtle import heading
import pygame
import numpy as np
from scipy.spatial.transform import Rotation as R
from icecream import ic as print
import time
import sys
import math
import cv2
from shapely.geometry import Polygon, Point

from informer import Informer
from proto.python_out import marker_msgs_pb2, geometry_msgs_pb2, path_msgs_pb2, cmd_msgs_pb2, ctrl_msgs_pb2
from utils import *
from time import sleep

import matplotlib.pyplot as plt
# read map
CAOLOU_MAP = pygame.image.load('./maps/map.png')
PLAYGROUND_MAP = pygame.image.load('./maps/playground.png')
DISPLAY_MAP = CAOLOU_MAP
sleep(1)
map_offset = np.array([0, 0])
fixed_goal = []
robot_dict = {}
bounding_box = dict()
car_number = None
robot_clicked_id = 1  #None
robot_select_id = []
box_clicked_id = None
ifm_dict = {}
rect_start_pos = None
rect_end_pos = None
# flags
use_caolou_map = True
use_playground_map = False
map_draging = False
goal_setting = False
robot_clicked = False
view_image = True
box_clicked = False
use_joystick = False
rect_select = False
recg_flag = False

temp_list = set()

class Receiver(object):
    def __init__(self, display_map):
        self.display_map = display_map
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
        global robot_dict
        while True:
            try:
                data, _ = self.path_sock.recvfrom(4096)
                data = data.decode("utf-8").split(';')
                robot_id, path = int(data[0]), data[1:]
                MAP_WIDTH, MAP_HEIGHT = self.display_map.get_size()
                offset = np.array([
                    WINDOW_WIDTH // 2 - MAP_WIDTH // 2,
                    WINDOW_HEIGHT // 2 - MAP_HEIGHT // 2
                ])
                path_pos = np.array([
                    np.array(
                        [float(pos.split(',')[0]),
                         float(pos.split(',')[1])]) + offset for pos in path
                    if pos != ''
                ])
                new_path_pos = np.array([
                    np.array(
                        [float(pos.split(',')[0]),
                         float(pos.split(',')[1])]) for pos in path
                    if pos != ''
                ])
                if robot_id in robot_dict.keys():
                    robot_dict[robot_id].update_path(path_pos)
                    robot_dict[robot_id].update_new_path(new_path_pos)
                # print(robot_id, path_pos, len(path_pos))
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


def parse_message(message, robot_id):
    # print('parse_message: ', len(message), robot_id)
    global bounding_box, use_caolou_map
    marker_list = marker_msgs_pb2.MarkerList()
    marker_list.ParseFromString(message)
    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    offset = np.array([
        WINDOW_WIDTH // 2 - MAP_WIDTH // 2,
        WINDOW_HEIGHT // 2 - MAP_HEIGHT // 2
    ])
    # print(marker_list.marker_list)
    for marker in marker_list.marker_list:
        try:
            # print(marker.pose.position.x, marker.pose.position.y)
            center_pos = gps2pixel(marker.pose.position.x, marker.pose.position.y, use_caolou_map) + offset
            orientation = R.from_quat([
                marker.pose.orientation.x, marker.pose.orientation.y,
                marker.pose.orientation.z, marker.pose.orientation.w
            ]).as_euler('xyz', degrees=False)[2]
            orientation += np.pi / 2
            height, width = 10 * marker.scale.x, 10 * marker.scale.y
            vertex_A = center_pos + height * np.array([
                np.cos(orientation), np.sin(orientation)
            ]) + width * np.array([-np.sin(orientation),
                                   np.cos(orientation)])
            vertex_B = center_pos + height * np.array([
                np.cos(orientation), np.sin(orientation)
            ]) - width * np.array([-np.sin(orientation),
                                   np.cos(orientation)])
            vertex_C = center_pos - height * np.array([
                np.cos(orientation), np.sin(orientation)
            ]) - width * np.array([-np.sin(orientation),
                                   np.cos(orientation)])
            vertex_D = center_pos - height * np.array([
                np.cos(orientation), np.sin(orientation)
            ]) + width * np.array([-np.sin(orientation),
                                   np.cos(orientation)])
            marker_id = marker.id
            new_box = np.array([vertex_A, vertex_B, vertex_C, vertex_D])
            # overlap filter
            overlap = False
            p1 = Polygon(new_box)
            for idx, pos in bounding_box.items():
                p2 = Polygon(pos)
                if p1.intersects(p2) and idx != marker_id:
                    overlap = True
                    break
            if not overlap:
                bounding_box[marker_id] = np.array(new_box)
        except:
            pass
        # print(bounding_box)


def parse_odometry(message, robot_id):
    global robot_dict, use_caolou_map
    # print('get odometry', len(message), robot_id)
    odometry = geometry_msgs_pb2.Pose()
    odometry.ParseFromString(message)
    # print(odometry)
    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    offset = np.array([
        WINDOW_WIDTH // 2 - MAP_WIDTH // 2,
        WINDOW_HEIGHT // 2 - MAP_HEIGHT // 2
    ])
    # offset = np.array([410, 180])
    # print(odometry.position.x-odometry.position.x%1, 100*(odometry.position.x%1), odometry.position.y-odometry.position.y%1, 100*(odometry.position.y%1))
    # new_la, new_lo = gps60to10(odometry.position.x-odometry.position.x%1, 100*(odometry.position.x%1), odometry.position.y-odometry.position.y%1, 100*(odometry.position.y%1))
    # print(new_la, new_lo)

    try:
        # print('get odometry', robot_id, odometry.position.x, odometry.position.y)
        robot_pos = gps2pixel(odometry.position.x, odometry.position.y, use_caolou_map) + offset
        # robot_pos = pixel2gps(odometry.position.x, odometry.position.y, use_caolou_map) + offset
        # print(robot_pos)
    except:
        return
    robot_heading = np.deg2rad(odometry.orientation.w)
    if robot_id in robot_dict.keys():
        robot_dict[robot_id].update_pos(robot_pos)
        robot_dict[robot_id].update_heading(robot_heading)
    else:
        # register new robot
        robot_dict[robot_id] = Robot(id=robot_id,
                                     pos=robot_pos,
                                     heading=robot_heading)

def parse_cmd(message, robot_id):
    global robot_dict
    cmd = cmd_msgs_pb2.Cmd()
    cmd.ParseFromString(message)
    robot_cmd = [[cmd.v, cmd.w]]
    if robot_id in robot_dict.keys():
        robot_dict[robot_id].update_cmd(robot_cmd)
    else:
        # register new robot
        robot_dict[robot_id] = Robot(id=robot_id,
                                     cmd=robot_cmd)


def parse_img(message, robot_id):
    global robot_dict
    # print('get img', len(message), 'id:', robot_id)
    nparr = np.frombuffer(message, np.uint8)
    robot_img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if robot_id in robot_dict.keys():
        robot_dict[robot_id].update_img(robot_img)
    else:
        # register new robot
        robot_dict[robot_id] = Robot(id=robot_id,
                                     img=robot_img)

def parse_recg(message, robot_id):
    global robot_dict, car_number
    print('parse_recg', message.decode(), robot_id)
    string = message.decode()
    if robot_id in robot_dict.keys():
        car_number = [robot_dict[robot_id].pos, string]
        print(car_number)
    # print(message, robot_id)

def send_path():
    global ifm_dict, robot_dict
    # print(len(path_list[:20]))
    for idx, robot in robot_dict.items():
        if len(robot.new_path_pos) > 1:
            path = path_msgs_pb2.Path()
            for i in range(len(robot.new_path_pos[:20])):
                pose = path_msgs_pb2.Pose2D()
                gps = pixel2gps(robot.new_path_pos[i][0], robot.new_path_pos[i][1], use_caolou_map)
                pose.x = gps[0]
                pose.y = gps[1]
                pose.theta = 0  #robot.new_path_pos[i][2]
                path.poses.append(pose)

            # print(path)
            sent_data = path.SerializeToString()
            # print('send', idx, len(sent_data))
            ifm_dict[idx].send_path(sent_data)

def send_ctrl(v, w, flag=1.):
    global ifm_dict
    ctrl_cmd = ctrl_msgs_pb2.Ctrl()
    ctrl_cmd.flag = flag
    ctrl_cmd.v = v
    ctrl_cmd.w = -w
    # print('send ctrl:', ctrl_cmd)
    sent_data = ctrl_cmd.SerializeToString()

    if robot_select_id is not None and len(robot_select_id) > 0:
        print('send ctrl', robot_select_id[0])
        ifm_dict[robot_select_id[0]].send_ctrl(sent_data)


def send_balloon_pos(alt, lon, robot_id):
    global ifm_dict, robot_dict
    ctrl_cmd = ctrl_msgs_pb2.Ctrl()
    ctrl_cmd.flag = 3
    ctrl_cmd.v = alt
    ctrl_cmd.w = lon
    # print('send_balloon_pos:', ctrl_cmd)
    sent_data = ctrl_cmd.SerializeToString()
    try:
        ifm_dict[robot_id].send_ctrl(sent_data)
    except:
        print('Fail to send balloon pos to robot_id:', robot_id)

def send_dect(w):
    global ifm_dict
    ctrl_cmd = ctrl_msgs_pb2.Ctrl()
    ctrl_cmd.flag = 2
    ctrl_cmd.v = 0
    ctrl_cmd.w = -w
    sent_data = ctrl_cmd.SerializeToString()

    if robot_select_id is not None and len(robot_select_id) > 0:
        print('send ctrl', robot_select_id[0])
        ifm_dict[robot_select_id[0]].send_ctrl(sent_data)
    else:
        print('No robot to send dect cmd !')

def decode_gps(bytes_array):
    gps = np.frombuffer(bytes_array, "float")
    lat = gps[0]
    lon = gps[1]
    return lat, lon

def parse_state(message, robot_id):
    global robot_dict, use_caolou_map, temp_list
    # print(robot_id)
    # print('get drone state', len(message), robot_id)
    # temp_list.add(robot_id)
    # print(temp_list, 'drone num:', len(temp_list))
    lat, lon = decode_gps(message)

    ###################################################
    # print(len(temp_list), robot_id)
    if robot_id == 10:
        # print('FFFFFFFFFFFFFFFFFFFFFFFF', lat, lon)
        send_balloon_pos(lat, lon, 18)
        send_balloon_pos(lat, lon, 19)

    ###################################################

    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    offset = np.array([
        WINDOW_WIDTH // 2 - MAP_WIDTH // 2,
        WINDOW_HEIGHT // 2 - MAP_HEIGHT // 2
    ])

    try:
        robot_pos = gps2pixel(lat, lon, use_caolou_map) + offset
    except:
        return

    robot_heading = 0#np.deg2rad(odometry.orientation.w)
    if robot_id in robot_dict.keys():
        robot_dict[robot_id].update_pos(robot_pos)
        robot_dict[robot_id].update_heading(robot_heading)
    else:
        # register new robot
        robot_dict[robot_id] = Robot(id=robot_id,
                                     pos=robot_pos,
                                     heading=robot_heading,
                                     is_drone = True)

class Cloud(Informer):
    def msg_recv(self):
        self.recv('msg', parse_message)

    def odm_recv(self):
        self.recv('odm', parse_odometry)

    def cmd_recv(self):
        self.recv('cmd', parse_cmd)

    def img_recv(self):
        self.recv('img', parse_img)
    
    def recg_recv(self):
        self.recv('recg', parse_recg)

    def send_path(self, message):
        self.send(message, 'path')

    def send_ctrl(self, message):
        self.send(message, 'ctrl')

    def state_recv(self):
        self.recv('state', parse_state)


def start_ifm():
    global ifm_dict
    for i in range(0, 20):
        ifm_dict[i] = Cloud(config='config.yaml', robot_id=i)


if __name__ == "__main__":
    pygame.init()
    SCREEN = pygame.display.set_mode(
        (WINDOW_WIDTH, WINDOW_HEIGHT))  #, pygame.RESIZABLE)
    pygame.display.set_caption('5G Monitor')
    icon = pygame.image.load('icon.png')
    pygame.display.set_icon(icon)
    CLOCK = pygame.time.Clock()
    SCREEN.fill(GREY)
    data_receiver = Receiver(DISPLAY_MAP)
    # 5G server setup
    start_thread = threading.Thread(target=start_ifm, args=())
    start_thread.start()
    # joystick setup
    joystick = setup_joystick()

    cnt = 0
    while True:
        start_time = time.time()
        cnt += 1
        SCREEN.fill(GREY)
        drawMaps(SCREEN, DISPLAY_MAP, map_offset)
        drawFixedGoal(SCREEN, fixed_goal, map_offset)
        drawGoal(SCREEN, robot_dict, map_offset)
        drawRobots(SCREEN, robot_dict, map_offset)
        drawBoundingBox(SCREEN, bounding_box, map_offset)
        drawCarNumber(SCREEN, car_number, map_offset)
        drawPath(SCREEN, robot_dict, map_offset)
        drawButton(SCREEN, use_caolou_map, use_playground_map, use_joystick)
        drawMessageBox(SCREEN, map_offset, robot_clicked, robot_clicked_id,
                       robot_dict, box_clicked, box_clicked_id, bounding_box)
        drawRectSelections(SCREEN, rect_select, rect_start_pos, rect_end_pos)

        if cnt % 5 == 0:
            # print('send path')
            send_path()

        for event in pygame.event.get():
            mods = pygame.key.get_mods()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN and mods & pygame.KMOD_CTRL:
                if event.button == 1:
                    map_draging = True
                    drag_start_pos = event.pos
            elif event.type == pygame.MOUSEBUTTONDOWN and mods & pygame.KMOD_ALT:
                if event.button == 1:
                    rect_select = True
                    rect_start_pos = event.pos
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # get mouse position
                mouse = pygame.mouse.get_pos()
                # button: set goal
                if BUTTON_GOAL_X <= mouse[
                        0] <= BUTTON_GOAL_X + BUTTON_WIDTH and BUTTON_GOAL_Y <= mouse[
                            1] <= BUTTON_GOAL_Y + BUTTON_HEIGHT:
                    goal_setting = True
                elif goal_setting:
                    goal_setting = False
                    robot_goal = mouse - map_offset
                    for robot_id in robot_select_id:
                        if robot_id in robot_dict.keys():
                            robot_dict[robot_id].update_goal(robot_goal)


                    goal_gps = pixel2gps(robot_goal[0], robot_goal[1], use_caolou_map)
                    print('img cor:', robot_goal, 'gps cor:', goal_gps, 'xy cor:', gps2xy(goal_gps[0], goal_gps[1]))
                    # for _ in range(10000):
                    #     send_balloon_pos(goal_gps[0], goal_gps[1], 19)
                    #     sleep(0.05)
                    # print('send over')
                    # MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
                    # print(robot_goal - np.array([
                    # WINDOW_WIDTH // 2 - MAP_WIDTH // 2,
                    # WINDOW_HEIGHT // 2 - MAP_HEIGHT // 2
                    # ]))
                # button: caolou map
                elif BUTTON_CAOLOU_X <= mouse[
                        0] <= BUTTON_CAOLOU_X + BUTTON_WIDTH and BUTTON_CAOLOU_Y <= mouse[
                            1] <= BUTTON_CAOLOU_Y + BUTTON_HEIGHT:
                    DISPLAY_MAP = CAOLOU_MAP
                    use_caolou_map = True
                    use_playground_map = False
                # button: playground map
                elif BUTTON_PLAYGROUND_X <= mouse[
                        0] <= BUTTON_PLAYGROUND_X + BUTTON_WIDTH and BUTTON_PLAYGROUND_Y <= mouse[
                            1] <= BUTTON_PLAYGROUND_Y + BUTTON_HEIGHT:
                    DISPLAY_MAP = PLAYGROUND_MAP
                    use_caolou_map = False
                    use_playground_map = True
                # button: joystick mode
                elif BUTTON_JOYSTICK_X <= mouse[
                        0] <= BUTTON_JOYSTICK_X + BUTTON_WIDTH and BUTTON_JOYSTICK_Y <= mouse[
                            1] <= BUTTON_JOYSTICK_Y + BUTTON_HEIGHT:
                    use_joystick = not use_joystick
                    if not use_joystick:
                        print('set zero')
                        for _ in range(10):
                            send_ctrl(1., 1., flag=0.)  # auto mode
                            time.sleep(0.01)
                elif BUTTON_DECT_L_X <= mouse[
                        0] <= BUTTON_DECT_L_X + BUTTON_WIDTH and BUTTON_DECT_L_Y <= mouse[
                            1] <= BUTTON_DECT_L_Y + BUTTON_HEIGHT:
                    
                    recg_flag = ~recg_flag
                    if recg_flag:
                        print('dect left !')
                        for _ in range(10):
                            send_dect(-0.1)
                            time.sleep(0.01)
                    else:
                        print('stop dect!')
                        for _ in range(10):
                            send_ctrl(1., 1., flag=0.)  # auto mode
                            time.sleep(0.01)
                
                elif BUTTON_DECT_R_X <= mouse[
                        0] <= BUTTON_DECT_R_X + BUTTON_WIDTH and BUTTON_DECT_R_Y <= mouse[
                            1] <= BUTTON_DECT_R_Y + BUTTON_HEIGHT:

                    recg_flag = ~recg_flag
                    if recg_flag:
                        print('dect right !')
                        for _ in range(10):
                            send_dect(0.1)
                            time.sleep(0.01)
                    else:
                        print('stop dect!')
                        for _ in range(10):
                            send_ctrl(1., 1., flag=0.)  # auto mode
                            time.sleep(0.01)

                # button: robot
                if robot_clicked:
                    # button: view image
                    BUTTON_IMAGE_X, BUTTON_IMAGE_Y = robot_dict[
                        robot_clicked_id].pos + map_offset + np.array(
                            [50, -125])
                    if BUTTON_IMAGE_X <= mouse[
                            0] <= BUTTON_IMAGE_X + BUTTON_WIDTH and BUTTON_IMAGE_Y <= mouse[
                                1] <= BUTTON_IMAGE_Y + BUTTON_HEIGHT:
                        view_image = True
                        print('show image')
                robot_clicked = False
                for idx, robot in robot_dict.items():
                    if robot.pos is not None and math.hypot(mouse[0] - (robot.pos + map_offset)[0],
                        mouse[1] - (robot.pos + map_offset)[1]) <= ROBOT_SIZE:
                        print('click robot {}'.format(idx))
                        robot_clicked = True
                        robot_clicked_id = idx
                        break
                # button: bounding box
                if box_clicked:
                    # button: get id
                    box_center = np.mean(bounding_box[box_clicked_id], axis=0)
                    BUTTON_ID_X, BUTTON_ID_Y = box_center + map_offset + np.array(
                        [50, -125])
                    if BUTTON_ID_X <= mouse[
                            0] <= BUTTON_ID_X + BUTTON_WIDTH and BUTTON_ID_Y <= mouse[
                                1] <= BUTTON_ID_Y + BUTTON_HEIGHT:
                        print('get id')
                box_clicked = False
                bounding_box_copy = bounding_box.copy()
                for idx, box in bounding_box_copy.items():
                    p1 = Point(mouse)
                    p2 = Polygon(box + map_offset)
                    if p2.contains(p1):
                        print('click box {}'.format(idx))
                        box_clicked = True
                        box_clicked_id = idx
                        break
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    map_draging = False
                    if rect_select:
                        p2 = Polygon(
                            np.array([
                                rect_start_pos,
                                (rect_start_pos[0], rect_end_pos[1]),
                                rect_end_pos,
                                (rect_end_pos[0], rect_start_pos[1])
                            ]))
                        robot_select_id = []
                        for idx, robot in robot_dict.items():
                            p1 = Point(robot.pos + map_offset)
                            if p2.contains(p1):
                                robot_select_id.append(robot.id)
                        print('select robot', robot_select_id)
                        rect_select = False
                        rect_start_pos = rect_end_pos = None
            elif event.type == pygame.MOUSEMOTION and mods & pygame.KMOD_CTRL:
                if map_draging:
                    drag_end_pos = event.pos
                    map_offset = map_offset + drag_end_pos - drag_start_pos
                    drag_start_pos = drag_end_pos
            elif event.type == pygame.MOUSEMOTION and mods & pygame.KMOD_ALT:
                if rect_select:
                    rect_end_pos = event.pos
            elif event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            elif event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

        # send goal
        if cnt % 10 == 0:
            sendGoal(DISPLAY_MAP, robot_dict, robot_select_id)

        # view image
        if view_image:
            try:
                # print(robot_img_dict.keys())
                for idx, robot in robot_dict.items():
                    if robot.img is not None:
                        cv2.imshow('Robot Image {}'.format(idx), robot.img)
                        if cv2.waitKey(25) & 0xFF == ord('q'):
                            view_image = False
                            cv2.destroyAllWindows()
            except:
                pass

        # parse joystick
        if use_joystick:
            try:
                steer, throttle, brake = parse_vehicle_wheel(joystick, CLOCK)
                if abs(steer) < 0.05: steer = 0.
                # print(steer, throttle, brake)
                drawJoystick(SCREEN, steer, throttle, brake)
                v = 2 * throttle if brake < 0.1 else 0.
                w = steer * 5.
                send_ctrl(v, w, flag=1.)  # manual ctrl
            except:
                pass

        pygame.display.update()
        CLOCK.tick(20)
        end_time = time.time()
        # print('frequency', 1/(end_time-start_time))
        if end_time - start_time < 1. / 30:
            time.sleep(1. / 30 - (end_time - start_time))
