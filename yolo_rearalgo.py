import random
import cv2
import numpy as np
from ultralytics import YOLO
import time

# Load class names from a file
my_file = open("/Users/adirajuadityasrivatsa/Documents/Programming/Python/yolo_project/coco.txt", "r")
class_list = my_file.read().split("\n")
my_file.close()

# Load a pretrained YOLOv8n model
model = YOLO("weights/yolov8n.pt", "v8")

# Constants for warning distance calculation
CAR_WIDTH = 1.8  # Average width of a car in meters
FOCAL_LENGTH = 800  # Approximate focal length in pixels (adjust for your camera)
ALPHA = 3.0  # Deceleration rate in m/s² (adjust based on real data)
TAU = 6.0  # Reaction time in seconds
R_MIN = 2.0  # Minimum safe distance in meters

# Set our car's speed in meters/second (200 km/h)
OUR_CAR_SPEED = 0 * 1000 / 3600  # 55.56 m/s

# Open video file or webcam
cap = cv2.VideoCapture("/Users/adirajuadityasrivatsa/Documents/Programming/Python/yolo_project/testcase2.MP4")

if not cap.isOpened():
    print("Cannot open video")
    exit()

# Variables to track previous frame's distance and time
prev_distance = None
prev_time = None
prev_velocity = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("Stream end. Exiting...")
        break

    # Perform object detection
    results = model.predict(source=[frame], conf=0.45, save=False)
    boxes = results[0].boxes
    current_time = time.time()  # Current time in seconds

    for i, box in enumerate(boxes):
        clsID = int(box.cls.numpy()[0])  # Class ID
        conf = box.conf.numpy()[0]  # Confidence
        bb = box.xyxy.numpy()[0]  # Bounding box coordinates [x1, y1, x2, y2]

        # Calculate bounding box width in pixels
        box_width_pixels = bb[2] - bb[0]

        # Estimate distance
        if box_width_pixels > 0:
            distance = (CAR_WIDTH * FOCAL_LENGTH) / box_width_pixels

            # Calculate velocity of the leading vehicle
            if prev_distance is not None and prev_time is not None:
                delta_distance = distance - prev_distance
                delta_time = current_time - prev_time
                v_F = delta_distance / delta_time  # Velocity of the leading car (in m/s)
            else:
                v_F = 0

            # Update previous frame's values
            prev_distance = distance
            prev_time = current_time

            # Calculate the critical warning distance (R_warning)
            v_L = OUR_CAR_SPEED  # Your car's speed
            R_warning = 0.5 * ((v_F**2 / ALPHA) - (v_L**2 / ALPHA)) + (v_F * TAU) + R_MIN

            # Convert R_warning to cm
            R_warning_cm = R_warning * 100
            distance_cm = distance * 100

            # Display speed and warning distance on the console
            print(f"Speed of Leading Vehicle: {v_F:.2f} m/s")
            print(f"Critical Warning Distance (R_warning): {R_warning_cm:.2f} cm")
            print(f"Measured Distance to Leading Vehicle: {distance_cm:.2f} cm")

            # Check if measured distance is less than the warning distance
            if distance_cm < R_warning_cm:
                distance_ratio = distance_cm / R_warning_cm  # Ratio of current distance to warning distance

                if distance_ratio < 0.7 and v_F >= 0.3:
                    light = "RED Light - High Collision Risk!"
                elif distance_ratio < 0.85 and v_F >= 0.2:
                    light = "YELLOW Light - Moderate Collision Risk!"
                else:
                    light = "GREEN Light - Low Collision Risk."
            else:
                light = "GREEN Light - Safe Distance."

            print(light)
            print("-" * 50)

            # Draw the light status on the frame
            font = cv2.FONT_HERSHEY_SIMPLEX
            if light == "RED Light - High Collision Risk!":
                color = (0, 0, 255)  # Red
            elif light == "YELLOW Light - Moderate Collision Risk!":
                color = (0, 255, 255)  # Yellow
            else:
                color = (0, 255, 0)  # Green

            cv2.putText(
                frame,
                light,
                (50, 50),  # Position of text (x, y)
                font,
                1,  # Font size
                color,
                2,  # Thickness
                cv2.LINE_AA,  # Line type
            )

    # Display the resulting frame
    cv2.imshow("Object Detection with Distance and Velocity", frame)

    # Exit when 'Q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release resources
cap.release()
cv2.destroyAllWindows()