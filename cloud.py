#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from os import wait
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

# read map
# BAIDU_MAP = pygame.image.load('./maps/yuquan.png')
BAIDU_MAP = pygame.image.load('./maps/map.png')
SATELLITE_MAP = pygame.image.load('./maps/satellite_map3.png')
DISPLAY_MAP = BAIDU_MAP
sleep(1)
map_offset = np.array([0, 0])
robot_goal = None
fixed_goal = []
robot_dict = {}
robot_img_dict = {}
bounding_box = dict()
path_pos = []
new_path_pos = []
robot_clicked_id = 1  #None
robot_select_id = []
box_clicked_id = None
ifm_dict = {}
rect_start_pos = None
rect_end_pos = None
# flags
use_baidu_map = True
use_satellite_map = False
map_draging = False
goal_setting = False
robot_clicked = False
view_image = True
box_clicked = False
use_joystick = False
rect_select = False
# file = open('gps.txt', 'w+')


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
        global path_pos, new_path_pos
        while True:
            try:
                data, _ = self.path_sock.recvfrom(4096)
                data = data.decode("utf-8").split(';')
                robot_id, path = data[0], data[1:]
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


def parse_message(message, robot_id):
    global bounding_box
    marker_list = marker_msgs_pb2.MarkerList()
    marker_list.ParseFromString(message)
    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    offset = np.array([
        WINDOW_WIDTH // 2 - MAP_WIDTH // 2,
        WINDOW_HEIGHT // 2 - MAP_HEIGHT // 2
    ])
    for marker in marker_list.marker_list:
        try:
            center_pos = gps2pixel(marker.pose.position.x,
                                   marker.pose.position.y) + offset
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
            for id, pos in bounding_box.items():
                p2 = Polygon(pos)
                if p1.intersects(p2) and id != marker_id:
                    overlap = True
                    break
            if not overlap:
                bounding_box[marker_id] = np.array(new_box)
        except:
            pass
        # print(bounding_box)


def parse_odometry(message, robot_id):
    global robot_dict
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
        # print(odometry)
        robot_pos = gps2pixel(odometry.position.x,
                              odometry.position.y) + offset
        # robot_pos = pixel2gps(odometry.position.x, odometry.position.y) + offset
        # print(robot_pos)
    except:
        return
    # print(odometry.position.x, odometry.position.y)
    # print(odometry)
    # robot_pos = gps2pixel(new_la, new_lo)# + offset
    # print('robot_pos', robot_pos)
    # robot_heading = R.from_quat([
    #     odometry.orientation.x, odometry.orientation.y, odometry.orientation.z,
    #     odometry.orientation.w
    # ]).as_euler('xyz', degrees=False)[2]
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


def parse_img(message, robot_id):
    global robot_dict, robot_img_dict
    # print('get img', len(message), 'id:', robot_id)
    nparr = np.frombuffer(message, np.uint8)
    robot_img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    # if robot_id in robot_dict.keys():
        # robot_dict[robot_id].update_img(robot_img)
    robot_img_dict[robot_id] = robot_img


def send_path(path_list):
    global ifm_dict
    # print(len(path_list[:20]))
    path = path_msgs_pb2.Path()
    for i in range(len(path_list[:20])):
        pose = path_msgs_pb2.Pose2D()
        gps = pixel2gps(*path_list[i])
        pose.x = gps[0]
        pose.y = gps[1]
        pose.theta = 0  #path_list[i][2]
        path.poses.append(pose)

    # print(path)
    sent_data = path.SerializeToString()
    # print('send', len(sent_data))
    if robot_select_id is not None:
        print('send path', robot_select_id[0])
        ifm_dict[robot_select_id[0]].send_path(sent_data)


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

class Cloud(Informer):
    def msg_recv(self):
        self.recv('msg', parse_message)

    def odm_recv(self):
        self.recv('odm', parse_odometry)

    def cmd_recv(self):
        self.recv('cmd', parse_cmd)

    def img_recv(self):
        self.recv('img', parse_img)

    def send_path(self, message):
        self.send(message, 'path')

    def send_ctrl(self, message):
        self.send(message, 'ctrl')


def start_ifm():
    global ifm_dict
    for i in range(1, 11):
        ifm_dict[i] = Cloud(config='config.yaml', robot_id=i)


# print(gps2pixel(30.259059, 120.120360))
# print(gps2pixel(30.265143, 120.122862))
# print(pixel2gps(807, 3450))
# print(pixel2gps(814, 632))

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
        drawGoal(SCREEN, robot_goal, map_offset)
        drawRobots(SCREEN, robot_dict, map_offset)
        drawBoundingBox(SCREEN, bounding_box, map_offset)
        drawPath(SCREEN, path_pos, map_offset)
        drawButton(SCREEN, use_baidu_map, use_satellite_map, use_joystick)
        drawMessageBox(SCREEN, map_offset, robot_clicked, robot_clicked_id,
                       robot_dict, box_clicked, box_clicked_id, bounding_box)
        drawRectSelections(SCREEN, rect_select, rect_start_pos, rect_end_pos)

        if len(path_pos) > 1 and cnt % 5 == 0:
            # print('send path')
            # send_path(path_pos)
            send_path(new_path_pos)

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
                    print(robot_goal)
                # button: baidu map
                elif BUTTON_BAIDU_X <= mouse[
                        0] <= BUTTON_BAIDU_X + BUTTON_WIDTH and BUTTON_BAIDU_Y <= mouse[
                            1] <= BUTTON_BAIDU_Y + BUTTON_HEIGHT:
                    DISPLAY_MAP = BAIDU_MAP
                    use_baidu_map = True
                    use_satellite_map = False
                # button: satellite map
                elif BUTTON_SATELLITE_X <= mouse[
                        0] <= BUTTON_SATELLITE_X + BUTTON_WIDTH and BUTTON_SATELLITE_Y <= mouse[
                            1] <= BUTTON_SATELLITE_Y + BUTTON_HEIGHT:
                    DISPLAY_MAP = SATELLITE_MAP
                    use_baidu_map = False
                    use_satellite_map = True
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
                    print('dect left !')
                    send_dect(0.1)
                
                elif BUTTON_DECT_R_X <= mouse[
                        0] <= BUTTON_DECT_R_X + BUTTON_WIDTH and BUTTON_DECT_R_Y <= mouse[
                            1] <= BUTTON_DECT_R_Y + BUTTON_HEIGHT:
                    print('dect right !')
                    send_dect(-0.1)

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
                    if math.hypot(mouse[0] -
                                  (robot.pos + map_offset)[0], mouse[1] -
                                  (robot.pos + map_offset)[1]) <= ROBOT_SIZE:
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
        if robot_goal is not None:
            if cnt % 10 == 0:
                sendGoal(DISPLAY_MAP, robot_dict, robot_select_id)

        # view image
        if view_image:
            try:
                # print(robot_img_dict.keys())
                for key in robot_img_dict.keys():
                    cv2.imshow('Robot Image', robot_img_dict[key])
                # cv2.imshow('Robot Image', robot_dict[robot_clicked_id].img)
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
