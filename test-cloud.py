#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from time import sleep
import numpy as np

from informer import Informer

def parse_img(message):
    print("Get img size:",len(message))
    nparr = np.frombuffer(message, np.uint8)
    img = cv2.imdecode(nparr,  cv2.IMREAD_COLOR)
    cv2.imshow('Image',img)
    cv2.waitKey(1)

class Cloud(Informer):
    def img_recv(self):
        self.recv('img', parse_img)

    def send_cmd(self, message):
        self.send(message, 'cmd')

if __name__ == '__main__':
    ifm = Cloud(config = 'config.yaml')
    while True:
        ifm.send_cmd(b'88888888888888')
        sleep(0.1)