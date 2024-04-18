import cv2
import numpy as np
import pytesseract
import math
import easyocr
import matplotlib.pyplot as plt

class Detection:
    def __init__(self,robot, timestep, move):
        self.robot=robot
        self.timestep=timestep
        self.camera = robot.getDevice('camera')
        self.colour_camera = robot.getDevice('colour_camera')
        self.compass = robot.getDevice('compass')  
        self.laser4 = robot.getDevice('laser4')

        self.camera.enable(timestep)
        self.colour_camera.enable(timestep)
        self.laser4.enable(timestep)

        pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'

# Tesseract configuration options
        self.TESSERACT_CONFIG = "--psm 6 --oem 3"
        self.reader = easyocr.Reader(['en'], gpu=True)  # Adjust 'gpu' based on your setup

# Set victim detection parameters
        self.MIN_CONTOUR_AREA = 10
        self.MAX_CONTOUR_AREA = 100    

# Initialize seen_victims list
        self.seen_victims = []
        self.seen_hazzards = []
        self.move = move



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
    
    
    # return center_x to main 
    def detect_victim_signs(self, image_data, img, camera):
        thresh = self.preprocess_image(image_data, camera)
        contours = self.detect_contours(thresh)

        for contour in contours:
            if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                roi = img[y:y+h, x:x+w]
                text = self.extract_text(roi)
                if text in ['U', 'H', 'S']:
                    center_x = x + w / 2
                    thr= 50
                    if (camera.getWidth() / 2)-center_x < thr and (camera.getWidth() / 2)-center_x >- thr:
                        self.move.stop()
                    elif(camera.getWidth() / 2)>center_x+thr:
                        self.move.left()
                    elif (camera.getWidth() / 2)<center_x+thr:
                        self.move.right()
                    else:
                        self.move.stop()
                    
                    gpsLoc=0
                    print(self.laser4.getValue())
                    print(f'Victim Detected: {text}')
                    self.seen_victims.append((text,gpsLoc))




    def detect_hazard_signs(self, image_data, img, camera):
        thresh = self.preprocess_image(image_data, camera)
        contours = self.detect_contours(thresh)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.MAX_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                roi = img[y:y+h, x:x+w]
                text = self.extract_text(roi)
                # Log detected hazards for visibility
                if text in ['CORROSIVE', 'FLAMMABLE GAS', 'ORGANIC PEROXIDE', 'POISON']:
                    print(f'Hazard Detected: {text}')
                    self.seen_hazzards.append(text)

        return self.seen_hazzards






    def detect_floor_color(self, image_data, camera):
        floor_value = 0

        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))

    
        height, width, _ = img.shape
        roi = img[int(height * 0.5):height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        brown_lower = np.array([36, 57.7, 58.4])
        brown_upper = np.array([45, 49.7, 61.6])
        black_lower = np.array([0, 0, 0])
        black_upper = np.array([180, 255, 30])
  

        brown_mask = cv2.inRange(hsv, brown_lower, brown_upper)
        black_mask = cv2.inRange(hsv, black_lower, black_upper)

    
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
        victims = self.detect_victim_signs(img_data, img, self.camera,)
        hazzards= self.detect_hazard_signs(img_data, img, self.camera)

        # Process the colour_camera image for floor color detection
        floor_img_data = self.colour_camera.getImage()
        floor = self.detect_floor_color(floor_img_data, self.colour_camera)

        return (victims,hazzards,floor)
