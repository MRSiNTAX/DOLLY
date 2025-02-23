# Dolly: A Voice-Activated And Gesture-Recognizing Smart Trashcan Utilizing The YOLO Algorithm For Autonomous Navigation

# Overview

DOLLY is a smart trashcan designed to improve waste disposal accessibility by integrating voice activation and gesture recognition. The system utilizes the YOLO (You Only Look Once) algorithm for real-time object detection, enabling it to recognize a 'thumbs-up' gesture and navigate autonomously towards the user. This project aims to assist individuals with mobility impairments and enhance waste management efficiency.

# Features

* Wake Word Detection: Uses Porcupine Wake Word Engine to activate the system with the phrase "Hey Dolly."
* Gesture Recognition: Implements YOLOv8 for detecting 'thumbs-up' gestures to initiate movement towards the user.
* Autonomous Navigation: Utilizes an ultrasonic sensor to detect obstacles and stop within a safe distance for trash disposal.
* Servo Motor Control: Controlled via an Arduino Uno for smooth movement and precise navigation.
* LED Indicators: Visual feedback for different system states (wake word detected, scanning, movement, and stop).

# Hardware Components

- Raspberry Pi 4 Model B - Runs YOLOv8 for real-time object detection.
- USB Webcam - Captures live video for gesture recognition.
- Arduino Uno - Controls motor movements based on Raspberry Pi commands.
- MG995 Servo Motors - Continuous rotation servos for wheel movement.
- HC-SR04 Ultrasonic Sensor - Measures distance to obstacles.
- LED Module - Provides system status indications.
