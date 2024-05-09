#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 11:20:17 2024

@author: yassine
"""

import threading
import cv2
import os
from ultralytics import YOLO
import numpy as np
import torch


os.environ["OPENCV_FFMPEG_READ_ATTEMPTS"] = "20000"


# def create_mask(height, width, bounding_boxes):
#     # Erstellen einer leeren Maske mit der gleichen Größe wie das Bild
#     mask = np.zeros((height, width), dtype=np.uint8)

#     # Füllen Sie jede Bounding Box in der Maske mit Einsen (1)
#     for box in bounding_boxes:
#         # Stellen Sie sicher, dass die angegebenen Koordinaten innerhalb der Grenzen des Bildes liegen
#         x = max(0, min(box[0], width - 1))
#         y = max(0, min(box[1], height - 1))
#         box_width = min(box[2], width - x)
#         box_height = min(box[3], height - y)

#         # Füllen Sie die Bounding Box in der Maske mit Einsen (1)
#         mask[y:y+box_height, x:x+box_width] = 255

#     return mask

def create_mask(height, width, bounding_boxes):
    # Erstellen einer leeren Maske mit der gleichen Größe wie das Bild
    mask = np.zeros((height, width), dtype=np.uint8)

    # Füllen Sie jede Bounding Box in der Maske mit Einsen (1)
    for box in bounding_boxes:
        # Stellen Sie sicher, dass die angegebenen Koordinaten innerhalb der Grenzen des Bildes liegen
        x1 = max(0, min(box[0], width - 1))
        y1 = max(0, min(box[1], height - 1))
        x2 = max(0, min(box[2], width - 1))
        y2 = max(0, min(box[3], height - 1))

        # Füllen Sie die Bounding Box in der Maske mit Einsen (1)
        mask[y1:y2+1, x1:x2+1] = 255

    return mask

def create_blank_mat(rows, cols, dtype=np.uint8):
    """
    Erstellt eine leere OpenCV-Matrix (cv::Mat) mit Nullen.
    
    Args:
    - rows: Anzahl der Zeilen in der Matrix.
    - cols: Anzahl der Spalten in der Matrix.
    - dtype: Datentyp für die Pixelwerte. Standardmäßig ist es np.uint8.

    Returns:
    - Eine OpenCV-Matrix (cv::Mat) gefüllt mit Nullen.
    """
    return np.zeros((rows, cols), dtype=dtype)

def run_tracker(filename, model, file_index, dirname):
    """
    Runs a video file or webcam stream concurrently with the YOLOv8 model using threading.

    This function captures video frames from a given file or camera source and utilizes the YOLOv8 model for object
    tracking. The function runs in its own thread for concurrent processing.

    Args:
        filename (str): The path to the video file or the identifier for the webcam/external camera source.
        model (obj): The YOLOv8 model object.
        file_index (int): An index to uniquely identify the file being processed, used for display purposes.

    Note:
        Press 'q' to quit the video display window.
    """
    video = cv2.VideoCapture(filename)  # Read the video file

    bboxes_video = []
   # out = cv2.VideoWriter('project.mp4',cv2.VideoWriter_fourcc(*'MP4V'), 30, (640,480))
    count = 0
    while True:
        ret, frame = video.read()  # Read the video frames
        print("GENERATING: " + str(dirname))
        
        # Exit the loop if no more frames in either video
        if not ret:
            break

        # Track objects in frames if available
        results = model.track(frame, persist=True)
        
        mask_flag = 1
        bboxes_person = []
        for idx, x in enumerate(results[0].boxes.cls):
            print("ID:::::" + str(x))
            if(x==0):
                mask_flag = 0
                #and (results[0].boxes.conf>=0.6)
                bboxes_person.append(results[0].boxes.xyxy[idx])
                print(bboxes_person)
            else:
                mask_flag = 1
        
        if(mask_flag == 0):
            bboxes = []            
            for i in bboxes_person:
                tlist = i.detach().cpu().numpy()
                bboxes.append([round(tlist[0]), round(tlist[1]), round(tlist[2]), round(tlist[3])])
                print(bboxes)
            
            mask = create_mask(480, 640, bboxes)
            print(mask)
        else:
            print("BLANK:::::::::::::::::::::::::::::::::::::::::::::::::::")
            mask = create_blank_mat(480, 640)

        
        filename = "./" + str(dirname) + "/" + str(count) + ".png"
        #mask_gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(filename, mask)
        count = count + 1
        #out.write(mask)
            
        # res_plotted = results[0].plot()
        # cv2.imshow(f"Tracking_Stream_{file_index}", res_plotted)

        # key = cv2.waitKey(1)
        # if key == ord('q'):
        #     break

    # Release video sources
   # out.release()
    video.release()
    
    


def run_tracker_in_thread(filename, model, file_index, dirname):
    """
    Runs a video file or webcam stream concurrently with the YOLOv8 model using threading.

    This function captures video frames from a given file or camera source and utilizes the YOLOv8 model for object
    tracking. The function runs in its own thread for concurrent processing.

    Args:
        filename (str): The path to the video file or the identifier for the webcam/external camera source.
        model (obj): The YOLOv8 model object.
        file_index (int): An index to uniquely identify the file being processed, used for display purposes.

    Note:
        Press 'q' to quit the video display window.
    """
    video = cv2.VideoCapture(filename)  # Read the video file

    bboxes_video = []
   # out = cv2.VideoWriter('project.mp4',cv2.VideoWriter_fourcc(*'MP4V'), 30, (640,480))
    count = 0
    while True:
        ret, frame = video.read()  # Read the video frames
        print("GENERATING: " + str(dirname))
        
        # Exit the loop if no more frames in either video
        if not ret:
            break

        # Track objects in frames if available
        results = model.track(frame, persist=True)
        
        mask_flag = 1
        bboxes_person = []
        for idx, x in enumerate(results[0].boxes.cls):
            print("ID:::::" + str(x))
            if(x==0):
                mask_flag = 0
                #and (results[0].boxes.conf>=0.6)
                bboxes_person.append(results[0].boxes.xyxy[idx])
                print(bboxes_person)
            else:
                mask_flag = 1
        
        if(mask_flag == 0):
            bboxes = []            
            for i in bboxes_person:
                tlist = i.detach().cpu().numpy()
                bboxes.append([round(tlist[0]), round(tlist[1]), round(tlist[2]), round(tlist[3])])
                print(bboxes)
            
            mask = create_mask(480, 640, bboxes)
            print(mask)
        else:
            print("BLANK:::::::::::::::::::::::::::::::::::::::::::::::::::")
            mask = create_blank_mat(480, 640)

        
        filename = "./" + str(dirname) + "/" + str(count) + ".png"
        #mask_gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        #cv2.imwrite(filename, mask)
        count = count + 1
        #out.write(mask)
            
        # res_plotted = results[0].plot()
        # cv2.imshow(f"Tracking_Stream_{file_index}", res_plotted)
        # key = cv2.waitKey(1)
        # if key == ord('q'):
        #     break

    # Release video sources
   # out.release()
    video.release()

# Load the models
model1 = YOLO('yolov8n.pt')
#model2 = YOLO('yolov8n-seg.pt')

# Define the video files for the trackers
video_file1 = "/home/yassine/Schreibtisch/video_output_left.mp4"  # Path to video file, 0 for webcam
video_file2 = "/home/yassine/Schreibtisch/video_output_right.mp4"
#video_file2 = 0  # Path to video file, 0 for webcam, 1 for external camera


run_tracker(video_file1, model1, 1, "masks_left")
run_tracker(video_file1, model1, 2, "masks_right")

# # Create the tracker threads
# tracker_thread1 = threading.Thread(target=run_tracker_in_thread, args=(video_file1, model1, 1, "masks_left"), daemon=False)
# tracker_thread2 = threading.Thread(target=run_tracker_in_thread, args=(video_file2, model1, 2, "masks_right"), daemon=False)

# # Start the tracker threads
# tracker_thread1.start()
# tracker_thread2.start()


# if (tracker_thread1.is_alive()):
#     print("THREAD LEFT ALIVE!!!!")
    
# if (tracker_thread2.is_alive()):
#     print("THREAD RIGHT ALIVE!!!!")

# # Wait for the tracker threads to finish
# tracker_thread1.join()
# tracker_thread2.join()

# # Clean up and close windows
# cv2.destroyAllWindows()