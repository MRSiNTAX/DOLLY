import pvporcupine
import pyaudio
import struct
import serial
import cv2
import time
import RPi.GPIO as GPIO  # Import GPIO for the ultrasonic sensor
from ultralytics import YOLO

# Set up the wake word path and model
ACCESS_KEY = "Ax90xE5Rua3eSzXkxjLbxCNOVlBMUHgSlhrxbz9/QEgvI1FYd+5dOQ=="
WAKE_WORD_PATH = "/home/sintax/dolly/wake_word/Hey-Dolly_en_raspberry-pi_v3_0_0.ppn"
YOLO_MODEL_PATH = "/home/sintax/dolly/models/dolly_model_mAP95.pt"

# Serial communication setup with Arduino
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# Ultrasonic Sensor GPIO Pins
TRIG_PIN = 23
ECHO_PIN = 24

# Setup GPIO pins for ultrasonic sensor
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Function to measure distance using the ultrasonic sensor
def measure_distance():
    # Send a pulse to trigger the sensor
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    # Record the time of start and arrival of the pulse
    start_time = time.time()
    stop_time = time.time()

    # Save the start time
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    # Save the arrival time
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    # Calculate the time difference
    elapsed_time = stop_time - start_time
    # Distance in cm (Speed of sound = 34300 cm/s)
    distance = (elapsed_time * 34300) / 2

    return distance

# Setup for wake word detection using Porcupine
def listen_for_wake_word(ser):
    porcupine = pvporcupine.create(access_key=ACCESS_KEY, keyword_paths=[WAKE_WORD_PATH])
    pa = pyaudio.PyAudio()
    audio_stream = pa.open(rate=porcupine.sample_rate, channels=1, format=pyaudio.paInt16, input=True, frames_per_buffer=porcupine.frame_length)
    print("Listening for wake word...")

    try:
        while True:
            pcm = audio_stream.read(porcupine.frame_length, exception_on_overflow=False)
            pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)
            keyword_index = porcupine.process(pcm)
            if keyword_index >= 0:
                print("Wake word detected! Activating Dolly...")
                ser.write(b"LED_WAKE\n")  # Turn on the LED immediately
                ser.flush()
                return True
    except KeyboardInterrupt:
        print("Terminated by user.")
    finally:
        audio_stream.stop_stream()
        audio_stream.close()
        pa.terminate()
        porcupine.delete()
    return False

# Function to perform object detection using YOLO and control Dolly's movement
def run_object_detection(ser):
    # Load the YOLO model
    model = YOLO(YOLO_MODEL_PATH)
    cap = cv2.VideoCapture(0)  # Open USB webcam

    time.sleep(1)  # Shorten the wait time for serial initialization

    # Signal the wake word detection and LED activation
    print("Wake word detected, LED should be ON for 3 seconds...")
    time.sleep(3)  # Keep the LED ON for 3 seconds

    # Start blinking the LED to indicate object detection mode
    ser.write(b"LED_BLINK\n")
    ser.flush()
    print("Object detection started, LED is blinking...")

    # Variable to keep track of current states
    motor_moving = False  # True if the motor is currently moving forward
    led_on = False  # True if LED is ON and stable (not blinking)

    try:
        while True:
            # Measure distance from the ultrasonic sensor
            distance = measure_distance()
            print(f"Measured Distance = {distance:.2f} cm")

            # Check if Dolly has reached the desired distance (30 cm or less)
            if distance <= 30:
                print("Desired distance reached. Stopping Dolly.")
                ser.write(b"STOP\n")  # Stop the motors
                ser.flush()
                break  # Terminate the script as the target distance is reached

            ret, frame = cap.read()
            if not ret:
                break

            # Run object detection on the frame
            results = model.predict(source=frame, imgsz=416, conf=0.5, show=True)

            # Check for 'thumbs_up' in the detected results
            detected = False
            for box in results[0].boxes:
                cls = results[0].names[int(box.cls[0])]
                if cls == "thumbs_up":
                    detected = True
                    break

            # Motor response: Move forward if 'thumbs up' is detected and distance is safe; otherwise, stop
            if detected and not motor_moving:
                print("Thumbs Up Detected! Moving forward...")
                ser.write(b"F_SLOW\n")  # Send the command to move forward
                ser.flush()
                motor_moving = True

                # Update LED state: Turn off LED (steady ON indicates detection)
                if not led_on:
                    ser.write(b"LED_OFF\n")  # Turn off blinking, indicating detection
                    ser.flush()
                    led_on = True  # Set LED state as OFF (indicating detection)

            elif not detected and motor_moving:
                print("No thumbs up detected, stopping...")
                ser.write(b"STOP\n")  # Send stop command to Arduino
                ser.flush()
                motor_moving = False

                # Resume LED blinking to indicate scanning mode
                if led_on:
                    ser.write(b"LED_BLINK\n")  # Resume blinking LED
                    ser.flush()
                    led_on = False

    except KeyboardInterrupt:
        print("Object detection terminated by user.")
    finally:
        cap.release()
        ser.write(b"LED_OFF\n")  # Turn off the LED when the script is stopped
        ser.flush()
        ser.close()
        GPIO.cleanup()  # Cleanup GPIO pins

if __name__ == "__main__":
    # Setup serial communication with Arduino
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(1)  # Shorten the wait time for serial initialization

    if listen_for_wake_word(ser):
        # Start the object detection process after the LED indication
        run_object_detection(ser)
        