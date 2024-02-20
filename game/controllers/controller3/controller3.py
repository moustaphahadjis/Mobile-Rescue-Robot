import cv2
import numpy as np
import pytesseract
import math
from controller import Robot, DistanceSensor

# Initialize robot and sensors
robot = Robot()
camera = robot.getDevice("camera")
camera.enable(32)  # Enable the camera with a time step of 32 ms
distance_sensor=robot.getDevice("distance_sensor")
distance_sensor.enable(32)

# Set the path to the Tesseract executable
pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'

# Tesseract configuration options
TESSERACT_CONFIG = "--psm 6 --oem 3"

# Set victim detection parameters
MIN_CONTOUR_AREA = 1000

# Function to preprocess the image
def preprocess_image(image_data):
    img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    img[:, :, 2] = np.zeros([img.shape[0], img.shape[1]])  # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]
    return thresh

# Function to detect contours
def detect_contours(thresh):
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

# Function to extract text using OCR
def extract_text(roi):
    text, _ = pytesseract.image_to_string(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB), config=TESSERACT_CONFIG)
    return text


# Function for visual victim detection
# Function for visual victim detection
def detect_visual_with_ocr(image_data, img):
    victims = []  # Create an empty list
    # Rest of your code

    thresh = preprocess_image(image_data)
    contours = detect_contours(thresh)

    detected_victim_types = set()  # Set to keep track of already detected victim types

    for c in contours:
        if cv2.contourArea(c) > MIN_CONTOUR_AREA:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            roi = img[y:y + h, x:x + w]
            text = extract_text(roi)

            if text in ['U', 'H', 'S'] and text not in detected_victim_types:
                print(f'{text.capitalize()} Victim Detected')
                victims.append((text, x, y))  # Append the victim type and coordinates to the list
                detected_victim_types.add(text)  # Mark the victim type as detected

    return victims  # Return the list of victims

detected_victims = []
def run():
    # Main loop
      # Initialize an empty list to store the detected victims
    while robot.step(32) != -1:
        img_data = camera.getImage()
        img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))

        victims = detect_visual_with_ocr(img_data, img)  # Get the list of victims from the function

        if victims:
            for v in victims:
                detected_victims.append(v)
                #print(f'{v[0]} Victim Detected at ({v[1]}, {v[2]})')
                print(distance_sensor.getValue())
        else:
            print("No victim detected")
        if( len(detected_victims)>0):
            break



def main():
    run()

    if len(detected_victims)>0:
        print("Detected Victims:")
        for v in detected_victims:
            print(f"{v[0]} at Processing at ({v[1]}, {v[2]})")
    else:
        print("No victims detected")


if __name__ == "__main__":
    main()

