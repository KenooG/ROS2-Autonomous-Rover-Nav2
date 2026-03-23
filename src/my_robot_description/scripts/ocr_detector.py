#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rover_interfaces.srv import OcrTask
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pytesseract
import numpy as np
import os
from imutils.object_detection import non_max_suppression

class TextRecognition:
    def __init__(self, east_model_path, min_confidence=0.5, width=320, height=320):
        self.east_model_path = east_model_path
        self.min_confidence = min_confidence
        self.width = width
        self.height = height
        
        self.net = cv2.dnn.readNet(self.east_model_path)
        self.layer_names = ['feature_fusion/Conv_7/Sigmoid', 'feature_fusion/concat_3']
        pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

    def recognize_text(self, image):
        orig_image = image.copy()
        orig_h, orig_w = orig_image.shape[:2]

        image_resized = cv2.resize(image, (self.width, self.height))
        ratio_w = orig_w / float(self.width)
        ratio_h = orig_h / float(self.height)

        blob = cv2.dnn.blobFromImage(image_resized, 1.0, (self.width, self.height), 
                                     (123.68, 116.78, 103.94), swapRB=True, crop=False)
        self.net.setInput(blob)
        scores, geometry = self.net.forward(self.layer_names)

        (num_rows, num_cols) = scores.shape[2:4]
        rectangles = []
        confidences = []

        for y in range(0, num_rows):
            scores_data = scores[0, 0, y]
            x_data_0 = geometry[0, 0, y]
            x_data_1 = geometry[0, 1, y]
            x_data_2 = geometry[0, 2, y]
            x_data_3 = geometry[0, 3, y]
            angles_data = geometry[0, 4, y]

            for x in range(0, num_cols):
                if scores_data[x] < self.min_confidence:
                    continue

                (offset_x, offset_y) = (x * 4.0, y * 4.0)
                angle = angles_data[x]
                cos = np.cos(angle)
                sin = np.sin(angle)
                h = x_data_0[x] + x_data_2[x]
                w = x_data_1[x] + x_data_3[x]

                end_x = int(offset_x + (cos * x_data_1[x]) + (sin * x_data_2[x]))
                end_y = int(offset_y - (sin * x_data_1[x]) + (cos * x_data_2[x]))
                start_x = int(end_x - w)
                start_y = int(end_y - h)

                rectangles.append((start_x, start_y, end_x, end_y))
                confidences.append(float(scores_data[x]))

        boxes = non_max_suppression(np.array(rectangles), probs=confidences)
        results = []

        for (start_x, start_y, end_x, end_y) in boxes:
            start_x = int(start_x * ratio_w)
            start_y = int(start_y * ratio_h)
            end_x = int(end_x * ratio_w)
            end_y = int(end_y * ratio_h)

            p = 5 
            start_x = max(0, start_x - p)
            start_y = max(0, start_y - p)
            end_x = min(orig_w, end_x + p)
            end_y = min(orig_h, end_y + p)

            roi = orig_image[start_y:end_y, start_x:end_x]

            if roi.size > 0:
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
               
                _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                
                config = '-l eng --oem 1 --psm 7'
                text = pytesseract.image_to_string(thresh, config=config)
                
                if text.strip():
                    results.append(text.strip().upper())

        return results

class OcrServer(Node):
    def __init__(self):
        super().__init__('ocr_detector_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "frozen_east_text_detection.pb")
        
        self.detector = TextRecognition(east_model_path=model_path)
        
        self.create_subscription(Image, '/model/perseverance/camera', self.image_callback, 10)
        self.srv = self.create_service(OcrTask, '/check_box_text', self.ocr_callback)
        
        self.get_logger().info("AI OCR (EAST + TESSERACT) READY! Waiting for signal...")

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def ocr_callback(self, request, response):
        if not request.start_scan:
            try: response.success = False
            except AttributeError: response.succes = False
            response.text = "NOT_TRIGGERED"
            return response

        self.get_logger().info("Scanning full frame using EAST model...")
        
        if self.latest_frame is None:
            try: response.success = False
            except AttributeError: response.succes = False
            response.text = "NO_IMAGE"
            return response

        found_texts = self.detector.recognize_text(self.latest_frame)
        
        is_food = False
        is_waste = False

        for text in found_texts:
            self.get_logger().info(f"EAST found text: '{text}'")
            if "FOOD" in text:
                is_food = True
            if "WASTE" in text:
                is_waste = True

        if is_food:
            response.text = "FOOD"
            try: response.success = True
            except AttributeError: response.succes = True
            self.get_logger().warn("CONFIRMED: FOOD")
        elif is_waste:
            response.text = "WASTE"
            try: response.success = True
            except AttributeError: response.succes = True
            self.get_logger().warn("CONFIRMED: WASTE")
        else:
            response.text = "NOTHING"
            try: response.success = False
            except AttributeError: response.succes = False
            self.get_logger().error("No keywords found in the frame.")

        return response

def main():
    rclpy.init()
    node = OcrServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()