# Import all relevant libraries
from controller import Robot
import math
import struct
import cv2
import pytesseract
import numpy as np

timeStep=32


useCV = False
try:
    import cv2
    import numpy as np
    useCV = True
    print("Camera-based visual victim detection is enabled.")
except:
    print("[WARNING] Since OpenCV and numpy is not installed, the visual victim detection is turned off. \
        Run 'pip install opencv-python' to install OpenCV and 'pip install numpy' on your terminal/command line.")

# Set the path to the Tesseract executable
pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'

# Tesseract configuration options
myconfig = "--psm 6 --oem 3"

# ... (rest of your code)
robot = Robot()


camera = robot.getDevice("camera")
camera.enable(timeStep)

camera_left= robot.getDevice("camera_left")
camera_left.enable(timeStep)

camera_right= robot.getDevice("camera_right")
camera_right.enable(timeStep)

gps=robot.getDevice('gps')
gps.enable(timeStep)


# Function to detect visual victims
def detectVisualWithOCR(image_data, camera, useCV=True):
    coords_list = []

    if useCV:
        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        img[:, :, 2] = np.zeros([img.shape[0], img.shape[1]])

        # Convert from BGR to HSV color space
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Apply threshold
        thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]

        # Draw all contours in green and accepted ones in red
        contours, h = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print('Contours',len(contours))
        
        for c in contours:
            if cv2.contourArea(c) > 1000:
                

                coords = list(c[0][0])
                coords_list.append(coords)
                print("Victim at x=" + str(coords[0]) + " y=" + str(coords[1]))

                # Extract text using OCR from the region of interest (ROI) around the detected contour
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

                
               
                roi = img[y:y + h, x:x + w]
                text = pytesseract.image_to_string(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB), config=myconfig)
                print("OCR Text:", text)
        

        

        #cv2.imshow('image', img)
        
        return coords_list

    else:
        return 0

#Object Detection    
       



# Main loop
while robot.step(timeStep) != -1:

    if useCV:
        # Get the image data from the camera
        img_data = camera.getImage()
        # Detect visual victims with OCR
        coords = detectVisualWithOCR(img_data, camera)

        print(gps.getValues())
        


    



        