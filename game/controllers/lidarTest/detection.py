import cv2
import numpy as np
import pytesseract
import math

class Detection:
    def __init__(self,robot, timestep):
        self.robot = robot
        self.timestep = timestep
        self.camera = robot.getDevice('camera')
        self.colour_camera = robot.getDevice('colour_camera')

        self.camera.enable(timestep)
        self.colour_camera.enable(timestep)

        pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'

# Tesseract configuration options
        self.TESSERACT_CONFIG = "--psm 6 --oem 3"

# Set victim detection parameters
        self.MIN_CONTOUR_AREA = 10

# Initialize seen_victims list
        self.seen_victims = []

# Function to preprocess the image for victim detection
    def preprocess_image(self,image_data, camera):
        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]
        return thresh

# Function to detect contours
    def detect_contours(self,thresh):
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

# Function to extract text using OCR
    def extract_text(self,roi):
        text = pytesseract.image_to_string(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB), config=self.TESSERACT_CONFIG).strip()
        return text

# Function for visual victim detection
    def detect_visual_with_ocr(self, image_data, img, camera):
        victims = []
        thresh = self.preprocess_image(image_data, camera)
        contours = self.detect_contours(thresh)

        for c in contours:
            if cv2.contourArea(c) > self.MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                roi = img[y:y + h, x:x + w]
                text = self.extract_text(roi)


        return victims

    def detect_floor_color(self, image_data, camera):
        floor_value = 0

        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))

    # Crop the image to focus on the likely floor area
        height, width, _ = img.shape
        roi = img[int(height * 0.5):height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        brown_lower = np.array([36, 57.7, 58.4])
        brown_upper = np.array([45, 49.7, 61.6])
        black_lower = np.array([0, 0, 0])
        black_upper = np.array([180, 255, 30])
  

        brown_mask = cv2.inRange(hsv, brown_lower, brown_upper)
        black_mask = cv2.inRange(hsv, black_lower, black_upper)

    # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        if cv2.countNonZero(brown_mask) > (roi.shape[0] * roi.shape[1] * 0.1):
            floor_value = 1
            print("Brown floor detected - reducing speed")
        elif cv2.countNonZero(black_mask) > (roi.shape[0] * roi.shape[1] * 0.1):
            floor_value = 2
            print("Black floor detected - avoiding hole")

        return floor_value

    def run(self):
      while self.robot.step(self.timestep) != -1:
        # Process images from all cameras for victim detection
    
        img_data = self.camera.getImage()
        img = np.array(np.frombuffer(img_data, np.uint8).reshape(( self.camera.getHeight(),  self.camera.getWidth(), 4)))
        victims = self.detect_visual_with_ocr(img_data, img, self.camera)

        # Process the colour_camera image for floor color detection
        floor_img_data = self.colour_camera.getImage()
        floor = self.detect_floor_color(floor_img_data, self.colour_camera)

        return (victims,floor)
