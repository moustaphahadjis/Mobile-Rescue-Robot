import cv2
import numpy as np
import pytesseract
import easyocr

class Detection:
    def __init__(self, robot, timestep, move):
        self.robot = robot
        self.timestep = timestep
        self.camera = robot.getDevice('camera')
        self.colour_camera = robot.getDevice('colour_camera')
        self.compass = robot.getDevice('compass')  
        self.laser4 = robot.getDevice('laser4')

        self.camera.enable(timestep)
        self.colour_camera.enable(timestep)
        self.laser4.enable(timestep)

        pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'
        self.TESSERACT_CONFIG = "--psm 6 --oem 3"
        self.reader = easyocr.Reader(['en'], gpu=True)  # Adjust 'gpu' based on your setup

        self.MIN_CONTOUR_AREA = 10
        self.MAX_CONTOUR_AREA = 100    

        self.hazard_keywords = {
            'FLAMMABLE GAS': ['FLAMMABLE', 'GAS', '2'],
            'CORROSIVE': ['CORROSIVE', '8'],
            'POISON': ['POISON', '6'],
            'ORGANIC PEROXIDE': ['ORGANIC', 'PEROXIDE', '5.2']
        }
        self.victim_keywords = {
            'Harmed Victim': ['H'],
            'Stable Victim': ['S'],
            'Unharmed Victim': ['U']
        }

        self.seen_hazards = {key: {} for key in self.hazard_keywords}
        self.hazard_updates = {key: {} for key in self.hazard_keywords}
        self.seen_victims = {key: {} for key in self.victim_keywords}
        self.victim_updates = {key: {} for key in self.victim_keywords}
        self.move = move

    def preprocess_image(self, image_data, camera):
        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]
        return thresh

    def detect_contours(self, thresh):
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def extract_victim(self, roi):
        results = pytesseract.image_to_string(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB), config=self.TESSERACT_CONFIG).strip()
        if results in ['H', 'S', 'U']:
            return [(results, 100)]  # Assuming maximum confidence if detected
        return []

    def extract_hazard(self, roi):
        results = self.reader.readtext(roi)
        return [(result[1].upper(), (result[2])*100) for result in results]

    def update_detected_hazards(self, text, confidence):
        for hazard, keywords in self.hazard_keywords.items():
            if text in keywords:
                current_conf = self.hazard_updates[hazard].get(text, 0)
                if confidence > current_conf:
                    self.hazard_updates[hazard][text] = confidence

    def update_detected_victims(self, text, confidence):
        for victim, keywords in self.victim_keywords.items():
            if text in keywords:
                current_conf = self.victim_updates[victim].get(text, 0)
                if confidence > current_conf:
                    self.victim_updates[victim][text] = confidence

    def print_hazards(self):
        for hazard, texts in self.hazard_updates.items():
            max_text, max_conf = max(texts.items(), key=lambda item: item[1], default=(None, None))
            if max_text:
                print(f"Hazard Detected: {hazard} ({max_text}) with confidence {max_conf:.2f}%")
        self.hazard_updates = {key: {} for key in self.hazard_keywords}

    def print_victims(self):
        for victim, texts in self.victim_updates.items():
            max_text, max_conf = max(texts.items(), key=lambda item: item[1], default=(None, None))
            if max_text:
                print(f"Victim Detected: {victim} ({max_text}) with confidence {max_conf:.2f}%")
        self.victim_updates = {key: {} for key in self.victim_keywords}  # Corrected reset


    def detect_victim_signs(self, image_data, img, camera):
        thresh = self.preprocess_image(image_data, camera)
        contours = self.detect_contours(thresh)

        for contour in contours:
            if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                roi = img[y:y+h, x:x+w]
                detected_texts = self.extract_victim(roi)

                for text, confidence in detected_texts:
                    self.update_detected_victims(text, confidence)

        return self.seen_victims

    def detect_hazard_signs(self, image_data, img, camera):
        thresh = self.preprocess_image(image_data, camera)
        contours = self.detect_contours(thresh)

        for contour in contours:
            if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                roi = img[y:y+h, x:x+w]
                detected_texts = self.extract_hazard(roi)

                for text, confidence in detected_texts:
                    self.update_detected_hazards(text, confidence)

        return self.seen_hazards


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
        victims = self.detect_victim_signs(img_data, img, self.camera,)
        hazzards= self.detect_hazard_signs(img_data, img, self.camera)

        # Process the colour_camera image for floor color detection
        floor_img_data = self.colour_camera.getImage()
        floor = self.detect_floor_color(floor_img_data, self.colour_camera)
        self.print_hazards()  # Print summarized hazard detections
        self.print_victims()  # Print summarized victim detections

        return (victims,hazzards,floor)
