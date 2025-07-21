"""Module for handling webcam input and processing.

This module provides functionality for capturing and processing video input
from webcam devices for robot vision applications.
"""

#!/usr/bin/env python3

import os

import cv2
import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

CAMERA_ID = int(os.getenv("CAMERA_ID", 0))
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
video_capture = cv2.VideoCapture(CAMERA_ID)
font = cv2.FONT_HERSHEY_SIMPLEX


for event in node:
    event_type = event["type"]
    if event_type == "INPUT":
        ret, frame = video_capture.read()
        if not ret:
            frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                f"No Webcam was found at index {CAMERA_ID}",
                (30, 30),
                font,
                0.75,
                (255, 255, 255),
                2,
                1,
            )
        node.send_output(
            "image",
            pa.array(frame.ravel()),
            event["metadata"],
        )
        cv2.imshow(str(CAMERA_ID), frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
video_capture.release()
