import cv2
import numpy as np
#import pytesseract
import easyocr
import matplotlib.pyplot as plt
import time

class Detection:
    def __init__(self, robot, timestep, move,map):
        self.map = map
        self.robot = robot
        self.timestep = timestep
        self.camera = robot.getDevice('camera')
        self.colour_camera = robot.getDevice('colour_camera')
        self.compass = robot.getDevice('compass')  
        self.laser4 = robot.getDevice('laser4')

        self.camera.enable(timestep)
        self.colour_camera.enable(timestep)
        self.laser4.enable(timestep)
        self.victims = self.robot.getFromDef('HUMANGROUP').getField('children')
        self.hazards = self.robot.getFromDef('HAZARDGROUP').getField('children')
        self.victims_count = self.victims.getCount()
        self.haz_count = self.hazards.getCount()
        #pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'
        #self.TESSERACT_CONFIG = "--psm 6 --oem 3"
        self.reader = easyocr.Reader(['en'], gpu=True)  # Adjust 'gpu' based on your setup

        self.MIN_CONTOUR_AREA = 10
        self.MAX_CONTOUR_AREA = 100    
        self.detected_signs = []

        self.sign_keywords = {
            'Flammable Gas': ['FLAMMABLE', 'GAS', '2'],
            'Corrosive': ['CORROSIVE', '8'],
            'Poison': ['POISON', '6'],
            'Organic Peroxide': ['ORGANIC', 'PEROXIDE', '5.2'],
            'Harmed Victim': ['H'],
            'Stable Victim': ['S'],
            'Unharmed Victim': ['U']
        }
        print(f'{self.victims_count} Victims present on this map')
        print(f'{self.haz_count} Hazards present on this map')
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
    
    def check(self, center):
        ini = 600
        fin = 700
        res = 0

        if center >ini and center <fin:
            res = 1
        elif center <ini and center <fin:
            res= 2
        else:
            res = 3
        
        return res




    def detect_signs(self, image_data, img, camera, keywords):
        thresh = self.preprocess_image(image_data, camera)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # Create a figure and an axes to plot on
        #fig, ax = plt.subplots()
        # Display the original image
        #ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

        midFrame = False
        for contour in contours:
            area = cv2.contourArea(contour)
            if 10000 < area < 40000:
                x, y, w, h = cv2.boundingRect(contour)
                roi = img[y:y+h, x:x+w]
                center_x = x + w / 2
                thr = w * 0.2

                # Display each ROI in a new figure to show text extraction
                #fig_roi, ax_roi = plt.subplots()
                #ax_roi.imshow(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB))
                #ax_roi.set_title('Region of Interest')

                detected_texts = self.extract_signs(roi)
                for detected_text in detected_texts:
                    for keyword_list in keywords.values():
                        if detected_text in keyword_list:
                            res = self.check(center_x)
                            t1 = time.time()
                            while self.robot.step(self.timestep) != -1:
                                t2 = time.time()
                                if t2 - t1 > 5:
                                    break
                                if res == 1:
                                    self.move.stop()
                                    midFrame = True
                                    break
                                elif res == 2:
                                    self.move.slow_left(self.map)
                                elif res == 3:
                                    self.move.slow_right(self.map)

                            #gps = self.map.detectVictimLoc(self.move.getOrientation())
                            gps = self.move.gps.getValues()
                            print(f'GPS: {gps}')
                            self.detected_signs.append(detected_text)
                            if self.victims_count>0:
                                for i in range(self.victims_count):
                                    try:
                                        loc =self.victims.getMFNode(i).getField('translation').getSFVec3f()
                                        print(loc)
                                        if (gps[0]< loc[0]+0.1 and gps[0]>loc[0]-0.1) or ( gps[2]< loc[2]+0.1 and gps[2]>loc[2]-0.1):
                                            print('Victim removed')
                                            self.victims.removeMF(i)
                                            self.victims_count = self.victims_count-1
                                    except:
                                        print('')
                            if self.haz_count>0:
                                for i in range(self.haz_count):
                                    try:
                                        loc =self.hazards.getMFNode(i).getField('translation').getSFVec3f()
                                        print(loc)
                                        if (gps[0]< loc[0]+0.1 and gps[0]>loc[0]-0.1) or ( gps[2]< loc[2]+0.1 and gps[2]>loc[2]-0.1):
                                            print('Hazard removed')
                                            self.hazards.removeMF(i)
                                            self.haz_count = self.haz_count - 1
                                    except:
                                        print('')
                            self.detected_signs.append(detected_text)
                            print(self.map.detectVictimLoc(self.move.getOrientation()))

                # Plot the bounding rectangle on the main image
                #rect = plt.Rectangle((x, y), w, h, edgecolor='red', facecolor='none')
                #ax.add_patch(rect)
                # Show the ROI plot
                #plt.show()

        # Show the main plot with the detected bounding boxes
        #plt.show()

        return self.detected_signs




    def detect_floor_color(self, image_data, camera):
        floor_value = 0

        # Convert the image data into a numpy array and reshape it according to the camera specifications
        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))

        # Convert the image from BGR to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define the HSV range for brown (commonly indicative of mud or dirt)
        brown_lower = np.array([10, 100, 20])  # Lower bound of brown color in HSV
        brown_upper = np.array([20,255,200])  # Upper bound of brown color in HSV

        
        black_lower = np.array([0, 0, 0])  
        black_upper = np.array([10, 10, 10])  

        # Create masks based on these color ranges
        brown_mask = cv2.inRange(hsv, brown_lower, brown_upper)
        black_mask = cv2.inRange(hsv, black_lower, black_upper)

       
        kernel = np.ones((5, 5), np.uint8)  #  morphological operations
        brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        if cv2.countNonZero(brown_mask) > (img.shape[0] * img.shape[1] * 0.1):
            floor_value = 1
            print("Swamp detected (Brown Area) - reducing speed")
        elif cv2.countNonZero(black_mask) > (img.shape[0] * img.shape[1] * 0.1):
            floor_value = 2
            print("Hole detected (Black Area) - avoiding ")
        else:
            floor_value = 0

        return floor_value

    def checkSigns(self, image_data, img, camera, keywords):
        thresh = self.preprocess_image(image_data, camera)
        contours = self.detect_contours(thresh)
        detected_signs = []

        val = False
        for contour in contours:
            if cv2.contourArea(contour) > 0 :
                #print(cv2.contourArea(contour))
                x, y, w, h = cv2.boundingRect(contour)
                roi = img[y:y+h, x:x+w]

                center_x = x + w / 2
                thr= w*0.2
                if self.check(center_x):
                    val = True
        return val

    def hasDetected(self):
            img_data = self.camera.getImage()
            img = np.array(np.frombuffer(img_data, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4)))
            return self.checkSigns(img_data, img, self.camera, self.sign_keywords)  # Pass the correct keywords

    def run(self):
        
        while self.robot.step(self.timestep) != -1:
            # Process images from the camera for sign detection
            img_data = self.camera.getImage()
            img = np.array(np.frombuffer(img_data, np.uint8).reshape(((self.camera.getHeight()), (self.camera.getWidth()), 4)))
            signs = self.detect_signs(img_data, img, self.camera, self.sign_keywords)  # Pass the correct keywords

            # Process the colour_camera image for floor color detection
            floor_img_data = self.colour_camera.getImage()
            floor = self.detect_floor_color(floor_img_data, self.colour_camera)  # Remove the extra argument
            self.print_signs(signs, self.sign_keywords)  # Print summarized sign detections

            return (signs, floor)

