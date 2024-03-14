import cv2
import numpy as np
import pytesseract
from controller import Robot

# Initialize robot and sensors
robot = Robot()
timestep = 32
camera = robot.getDevice("camera")
camera_right = robot.getDevice("camera_right")
camera_left = robot.getDevice("camera_left")
colour_camera = robot.getDevice("colour_camera")

camera.enable(timestep)  # Enable the cameras with a time step of 32 ms
camera_right.enable(timestep)
camera_left.enable(timestep)
colour_camera.enable(timestep)

# Set the path to the Tesseract executable
pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'

# Tesseract configuration options
TESSERACT_CONFIG = "--psm 6 --oem 3"

# Set victim detection parameters
MIN_CONTOUR_AREA = 10

# Initialize seen_victims list
seen_victims = []

# Function to preprocess the image for victim detection
def preprocess_image(image_data, camera):
    img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]
    return thresh

# Function to detect contours
def detect_contours(thresh):
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

# Function to extract text using OCR
def extract_text(roi):
    text = pytesseract.image_to_string(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB), config=TESSERACT_CONFIG).strip()
    return text

# Function for visual victim detection
def detect_visual_with_ocr(image_data, img, camera):
    victims = []
    thresh = preprocess_image(image_data, camera)
    contours = detect_contours(thresh)

    for c in contours:
        if cv2.contourArea(c) > MIN_CONTOUR_AREA:
            x, y, w, h = cv2.boundingRect(c)
            roi = img[y:y + h, x:x + w]
            text = extract_text(roi)

            if text in ['U', 'H', 'S']:
                # Create a unique identifier for the victim
                victim_id = f'{text}_{x}_{y}'

                # Check if the victim has been seen before
                if victim_id not in seen_victims:
                    print(f'{text.capitalize()} Victim Detected')
                    seen_victims.append(victim_id)
    return victims

def detect_floor_color(image_data, camera):
    img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))

    # Crop the image to focus on the likely floor area
    height, width, _ = img.shape
    roi = img[int(height * 0.5):height, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    brown_lower = np.array([10, 100, 20])
    brown_upper = np.array([20, 255, 200])
    black_lower = np.array([0, 0, 0])
    black_upper = np.array([180, 255, 30])

    brown_mask = cv2.inRange(hsv, brown_lower, brown_upper)
    black_mask = cv2.inRange(hsv, black_lower, black_upper)

    # Apply morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

    if cv2.countNonZero(brown_mask) > (roi.shape[0] * roi.shape[1] * 0.1):
        print("Brown floor detected - reducing speed")
    elif cv2.countNonZero(black_mask) > (roi.shape[0] * roi.shape[1] * 0.1):
        print("Black floor detected - avoiding hole")

def run():
    while robot.step(timestep) != -1:
        # Process images from all cameras for victim detection
        for cam in [camera, camera_left, camera_right]:
            img_data = cam.getImage()
            img = np.array(np.frombuffer(img_data, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4)))
            detect_visual_with_ocr(img_data, img, cam)

        # Process the colour_camera image for floor color detection
        floor_img_data = colour_camera.getImage()
        detect_floor_color(floor_img_data, colour_camera)

def main():
    run()

if __name__ == "__main__":
    main()
