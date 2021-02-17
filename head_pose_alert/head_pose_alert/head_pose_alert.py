#!/usr/bin/python3
# -*- coding:utf-8 -*-

import cv2
import argparse

import os
import time
from collections import deque

from . import service

import rclpy

from std_msgs.msg import Int16MultiArray, Float32MultiArray
#from auv_msgs.msg import RPY

alert_check = deque(maxlen=90)

class headposeAlert:
    def __init__(self):
        self.node = rclpy.create_node("head_pose_node")

        self.node.declare_parameter('video_input', 0)
        self.node.declare_parameter('threshold', [-35, 35, 10, -10])

        self.video_input = self.node.get_parameter('video_input')
        self.threshold = self.node.get_parameter('threshold')

        self.rpyPub = self.node.create_publisher(Float32MultiArray, '/headpose/rpy')
        self.alertPub = self.node.create_publisher(Int16MultiArray, '/headpose/alert')

        self.rpyMsg = Float32MultiArray()
        self.rpyMsg.data = [0.0] * 3

        self.alertMsg = Int16MultiArray()
        self.alertMsg.data = [0] * 2


        cwd = os.getcwd()
        cwd = cwd + "/src/head_pose_alert/head_pose_alert/weights/"

        self.fd = service.UltraLightFaceDetecion(cwd + "RFB-320.tflite",
                                            conf_threshold=0.95)
        
        self.fa = service.DepthFacialLandmarks(cwd + "sparse_face.tflite")
        
        self.handler = getattr(service, "pose")

        self.cap = cv2.VideoCapture(self.video_input._value)

        while True:
            self.publishAlert()


    def publishAlert(self):
        global alert_check

        (l_thr, r_thr, u_thr, d_thr) = self.threshold._value
        
        ret, frame = self.cap.read()

        # face detection
        boxes, scores = self.fd.inference(frame)

        # raw copy for reconstruction
        feed = frame.copy()

        for results in self.fa.get_landmarks(feed, boxes):
            pyr = self.handler(frame, results, (224, 255, 255))

            #None : 0, Left : 1, Right : 2
            lr = 0
            if pyr[1] <= l_thr:
                lr = 1
            elif pyr[1] >= r_thr:
                lr = 2

            #None : 0, Up : 1, Down : 2
            ud = 0
            if pyr[0] >= u_thr:
                ud = 1
            elif pyr[0] <= d_thr:
                ud = 2

            #None : 0, Left : 1, Right : 2, Up : 3, Down : 4, Left&Up : 5, Left&Down : 6, Right&Up : 7, Right&Down : 8
            if lr == 0 and ud == 0:
                alert_num = 0
            elif lr == 1 and ud == 0:
                alert_num = 1        
            elif lr == 2 and ud == 0:
                alert_num = 2      
            elif lr == 0 and ud == 1:
                alert_num = 3      
            elif lr == 0 and ud == 2:
                alert_num = 4      
            elif lr == 1 and ud == 1:
                alert_num = 5      
            elif lr == 1 and ud == 2:
                alert_num = 6      
            elif lr == 2 and ud == 1:
                alert_num = 7      
            elif lr == 2 and ud == 2:
                alert_num = 8      

            #None : 0, Warn : 1
            alert_warn = 0            
            alert_check.append(alert_num)
            
            if len(alert_check) == 90 and alert_check.count(0) <= 10 :
                alert_warn = 1

            self.rpyMsg.data = [pyr[2], pyr[0], pyr[1]]
            self.alertMsg.data = [alert_num, alert_warn]

            self.rpyPub.publish(self.rpyMsg)
            self.alertPub.publish(self.alertMsg)

            '''
            cv2.imshow("demo", frame)
            if cv2.waitKey(1) == ord("q"):
                break
            '''

def main():
    rclpy.init()
    headposeAlert()
    

#def main(args, color=(224, 255, 255)):
"""
def main(color=(224, 255, 255)):

    cwd = os.getcwd()
    cwd = cwd + "/src/head_pose_alert/head_pose_alert/weights/"

    fd = service.UltraLightFaceDetecion(cwd + "RFB-320.tflite",
                                        conf_threshold=0.95)
    
    fa = service.DepthFacialLandmarks(cwd + "sparse_face.tflite")
    
    handler = getattr(service, "pose")
    
    '''
    try:
        input_video = int(args.video)

    except ValueError:
        input_video = args.video
    '''

    input_video = 0 
    cap = cv2.VideoCapture(input_video)
    
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        # face detection
        boxes, scores = fd.inference(frame)

        # raw copy for reconstruction
        feed = frame.copy()

        for results in fa.get_landmarks(feed, boxes):
            pyr = handler(frame, results, color)
            print(pyr)

        '''
        cv2.imshow("demo", frame)
        if cv2.waitKey(1) == ord("q"):
            break
        '''
"""

if __name__ == "__main__":
    main()
    #parser = argparse.ArgumentParser(description="Video demo script.")
    #parser.add_argument("-v", "--video", type=str, required=True)

    #args = parser.parse_args()

