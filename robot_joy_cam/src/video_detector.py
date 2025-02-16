#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

from ultralytics import YOLO

bridge = CvBridge()

model = YOLO("yolo11n.pt")

last_img = None

def image_callback(msg):
    global last_img
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is None:
            rospy.logwarn("Ошибка: получено пустое изображение!")
            return
        last_img = image    
        
    except Exception as e:
        rospy.logerr(f"Ошибка OpenCV: {e}")

def main():
    rospy.init_node("compressed_image_subscriber", anonymous=True)
    rospy.Subscriber("/camera/image/compressed", CompressedImage, image_callback, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if last_img is not None:
            results = model(last_img, classes=[39])
            result_image = results[0].plot()
            cv2.imshow("result_image", result_image)
            cv2.waitKey(1)
        rate.sleep()
    
    
if __name__ == "__main__":
    main()
