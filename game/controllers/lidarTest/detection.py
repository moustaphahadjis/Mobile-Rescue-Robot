import cv2
import numpy as np
#import pytesseract
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

        #pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'
        #self.TESSERACT_CONFIG = "--psm 6 --oem 3"
        self.reader = easyocr.Reader(['en'], gpu=True)  # Adjust 'gpu' based on your setup

        self.MIN_CONTOUR_AREA = 10
        self.MAX_CONTOUR_AREA = 100    

        self.sign_keywords = {
            'Flammable Gas': ['FLAMMABLE', 'GAS', '2'],
            'Corrosive': ['CORROSIVE', '8'],
            'Poison': ['POISON', '6'],
            'Organic Peroxide': ['ORGANIC', 'PEROXIDE', '5.2'],
            'Harmed Victim': ['H'],
            'Stable Victim': ['S'],
            'Unharmed Victim': ['U']
        }

        self.move = move

    def preprocess_image(self, image_data, camera):
        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]
        return thresh

    def detect_contours(self, thresh):
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def extract_signs(self, roi):
        results = self.reader.readtext(roi)
        return [result[1].upper() for result in results]

    def print_signs(self, detected_signs, keywords):
        for category, keyword_list in keywords.items():
            for detected_text in detected_signs:
                if detected_text in keyword_list:
                    print(f"{category} Detected")

    def detect_signs(self, image_data, img, camera, keywords):
        thresh = self.preprocess_image(image_data, camera)
        contours = self.detect_contours(thresh)
        detected_signs = []

        midFrame = False
        for contour in contours:
            if cv2.contourArea(contour) > 6000 and cv2.contourArea(contour) < 10000 :
                print(cv2.contourArea(contour))
                x, y, w, h = cv2.boundingRect(contour)
                roi = img[y:y+h, x:x+w]

                center_x = x + w / 2
                thr= 50
                if (camera.getWidth() / 2)-center_x < thr and (camera.getWidth() / 2)-center_x >- thr:
                    self.move.stop()
                elif(camera.getWidth() / 2)>center_x+thr:
                    self.move.left()
                elif (camera.getWidth() / 2)<center_x+thr:
                    self.move.right()
                else:
                    midFrame = True
                    self.move.stop()


                print(self.laser4.getValue())
                if midFrame:
                    detected_texts = self.extract_signs(roi)
                    for detected_text in detected_texts:
                        for keyword_list in keywords.values():
                            if detected_text in keyword_list:
                                detected_signs.append(detected_text)

        return detected_signs


    def detect_floor_color(self, image_data, camera):
        floor_value = 0

        # Convert the image data into a numpy array and reshape it according to the camera specifications
        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))

        # Convert the image from BGR to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define the HSV range for brown (commonly indicative of mud or dirt)
        brown_lower = np.array([36, 57.7, 58.4])  # Lower bound of brown color in HSV
        brown_upper = np.array([45, 49.7, 61.6])  # Upper bound of brown color in HSV

        # Define the HSV range for black (potentially hazardous areas or holes)
        black_lower = np.array([0, 0, 0])  # Lower bound of black color in HSV
        black_upper = np.array([180, 255, 30])  # Upper bound of black color in HSV

        # Create masks based on these color ranges
        brown_mask = cv2.inRange(hsv, brown_lower, brown_upper)
        black_mask = cv2.inRange(hsv, black_lower, black_upper)

        # Apply morphological operations to reduce noise in the masks
        kernel = np.ones((5, 5), np.uint8)  # Kernel for morphological operations
        brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        # Determine if significant portions of the floor are brown or black by comparing the non-zero values in masks
        if cv2.countNonZero(brown_mask) > (img.shape[0] * img.shape[1] * 0.1):
            floor_value = 1
            print("Brown floor detected - reducing speed")
        elif cv2.countNonZero(black_mask) > (img.shape[0] * img.shape[1] * 0.1):
            floor_value = 2
            print("Black floor detected - avoiding hole")

        return floor_value


    def run(self):
        while self.robot.step(self.timestep) != -1:
            # Process images from the camera for sign detection
            img_data = self.camera.getImage()
            img = np.array(np.frombuffer(img_data, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4)))
            signs = self.detect_signs(img_data, img, self.camera, self.sign_keywords)  # Pass the correct keywords

            # Process the colour_camera image for floor color detection
            floor_img_data = self.colour_camera.getImage()
            floor = self.detect_floor_color(floor_img_data, self.colour_camera)  # Remove the extra argument
            self.print_signs(signs, self.sign_keywords)  # Print summarized sign detections

            return (signs, floor)

