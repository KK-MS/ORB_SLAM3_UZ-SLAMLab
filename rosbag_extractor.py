#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  6 11:27:49 2024

@author: yassine
"""

import cv2
import os
import rospy
import rosbag
from cv_bridge import CvBridge

STEREO = True


bag_file = "/home/yassine/aufnahme_marker_neu13.bag"

output_video_left = "/home/yassine/Schreibtisch/video_output_left.mp4"
output_video_right = "/home/yassine/Schreibtisch/video_output_right.mp4"

print("starting...\n")

# Initialisiere den CvBridge
bridge = CvBridge()

# Öffne die ROS-Bag-Datei
bag = rosbag.Bag(bag_file, 'r')

# Definiere die Video-Aufnahme*
fourcc = cv2.VideoWriter_fourcc(*'MP4V')
out_left = cv2.VideoWriter(output_video_left, fourcc, 30.0, (640, 480)) # Ändere die Auflösung und Bildrate nach Bedarf
out_right = cv2.VideoWriter(output_video_right, fourcc, 30.0, (640, 480)) # Ändere die Auflösung und Bildrate nach Bedarf

# Iteriere durch die Nachrichten in der ROS-Bag
for topic, msg, t in bag.read_messages():
    # Überprüfe, ob es sich bei der Nachricht um ein Bild handelt
    print(t)
    if topic == '/camera/infra1/image_rect_raw':
        try:
            # Wandle das ROS-Bild in ein OpenCV-Bild um
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Schreibe das Bild in das Video
            out_left.write(cv_image)

        except Exception as e:
            print(e)
            continue
    if topic == '/camera/infra2/image_rect_raw':
        try:
            # Wandle das ROS-Bild in ein OpenCV-Bild um
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Schreibe das Bild in das Video
            out_right.write(cv_image)

        except Exception as e:
            print(e)
            continue


# Schließe die Aufnahme
out_left.release()
out_right.release()

# Schließe die ROS-Bag
bag.close()
