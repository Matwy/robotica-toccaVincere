import sys
import time

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

import cvtools

ALTEZZA, LARGHEZZA = None, None

model = 'giugia_pestami_le_palle.tflite'

# Initialize the object detection model
base_options = core.BaseOptions(file_name=model, use_coral=False, num_threads=4)
detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
detector = vision.ObjectDetector.create_from_options(options)

output = None
Y_ABBASSA_BRACCIO = 35
Y_DENTRO_BECCO = 50

def get_selected_ball(frame):
    global output
    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_tensor = vision.TensorImage.create_from_array(rgb_image)
    detection_result = detector.detect(input_tensor)
    
    ball = []
    for d in detection_result.detections:
        x, y, w, h = d.bounding_box.origin_x, d.bounding_box.origin_y, d.bounding_box.width, d.bounding_box.height
        ball.append((x,y,w,h))
        cv2.rectangle(output, (x,y), (x+w, y+h), (255,0,0), 1)
    
    if len(ball) > 0:
        ball.sort(key=lambda b : b[2]*b[3], reverse=True) # ordina le palle in base alla più grande (quindi la più vicina)
        x, y, w, h = ball[0]
        cv2.rectangle(output, (x,y), (x+w, y+h), (255,0,255), 4)
        return ball[0]

def raccogli_palla(robot):
    cv2.destroyAllWindows()
    
    while True:
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            robot.servo.deinit_pca()
            cv2.destroyAllWindows()
            break

def ez(robot):
    global output, LARGHEZZA, ALTEZZA
    
    cv2.destroyAllWindows()
    robot.motors.motors(0, 0)
    robot.servo.cam_EZ()
    robot.camstream_EZ()
    time.sleep(0.5)
    LARGHEZZA, ALTEZZA = robot.cam_stream.camera.resolution
    pinza_su = True
    while True:
        frame = robot.get_frame()
        output = frame.copy()
        
        ball = get_selected_ball(frame)
        if ball:
            print("[EZ] palla ", ball)
            speed = 40 if ball[1] < Y_ABBASSA_BRACCIO else 20
            errore_x = ball[0] - ALTEZZA//2
            robot.motors.motors(speed + (errore_x), speed - (errore_x))
            
            if ball[1] > Y_ABBASSA_BRACCIO and pinza_su:
                robot.motors.motors(0, 0)
                robot.servo.pinza_giu()
                robot.servo.becco_aperto()
                pinza_su = False
            elif ball[1] <= Y_ABBASSA_BRACCIO and not pinza_su:
                print("pinza giu")
                robot.motors.motors(0, 0)
                robot.servo.pinza_su()
                pinza_su = True
            
            if ball[1] > 50:
                robot.servo.becco_chiuso()
                time.sleep(0.3)
                robot.motors.motors(-20, -20)
                robot.servo.pinza_su()
                time.sleep(1)
                robot.servo.becco_molla_morti()
                
        else:
            robot.motors.motors(0, 0)
        
        cv2.imshow("output", output)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            robot.servo.deinit_pca()
            cv2.destroyAllWindows()
            break