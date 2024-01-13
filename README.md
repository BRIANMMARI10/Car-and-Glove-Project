## INTRODUCTION
The objective of this project IS to redesign the autonomous car as well as make a left-handed glove that can help the user control the car’s motion. The glove can be used to control the direction of the car as well as the current state of the
car(stationary/motion) but not change the speed of the car while in motion. The car also has an extra fun feature which allows the user control the movements of the Raspberry camera wirelessly from your laptop/phone. 

## KEY SUBSYSTEMS - HARDWARE
1. Car: H-Bridge
     Power Board:
     ESP32 Board:
     Raspberry Pi Zero + Raspberry Pi Camera + LCD Display
2. Glove: • Breadboard
            The 2nd ESP32(Glove ESP32)
            Flex Sensors + Pressure Sensors + Spirit level

## app_move.py & appV2.py & camera_pi.py
Python files that enable the user to move the Raspberry camera from a laptop/phone. The user has to open the custom-created web server and connect to a wifi(personally named -Isabella) to control the camera.

## car.ino & glove.ino
Arduino files that contains code for controlling the movement of the car depending on the state of the glove.

## link to the full-details of each component
https://github.com/BRIANMMARI10/Car-and-Glove-Project/blob/44419830976d3d623cc1e700185d7eb9c450751a/FINAL%20PROJECT%20REPORT.pdf

