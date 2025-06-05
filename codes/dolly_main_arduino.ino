#include <Servo.h>

// Servo pins
const int leftServoPin = 9;    // Left servo pin
const int rightServoPin = 10;  // Right servo pin
const int ledPin = 2;          // LED pin for visual indicator

// Ultrasonic sensor pins
const int trigPin = A1; // Trigger pin
const int echoPin = A2; // Echo pin

// Create Servo objects
Servo leftServo;
Servo rightServo;

// Variables for LED state management
unsigned long ledTimer = 0;
const unsigned long blinkInterval = 500;  // Interval for LED blinking (500 ms)
bool ledState = false;                    // Keeps track of the LED's on/off state
bool isBlinking = false;                  // Keeps track if the LED should blink
bool ledWakeActive = false;               // To track if LED is in wake state

void setup() {
    Serial.begin(9600);       // Ensure baud rate matches Raspberry Pi serial communication
    leftServo.attach(leftServoPin);
    rightServo.attach(rightServoPin);
    pinMode(ledPin, OUTPUT);  // Set LED pin as OUTPUT
    pinMode(trigPin, OUTPUT); // Set trig pin as OUTPUT
    pinMode(echoPin, INPUT);  // Set echo pin as INPUT
    digitalWrite(ledPin, LOW);
    Serial.println("Arduino Ready!");
}

void loop() {
    // Handle incoming serial commands
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        Serial.print("Received command: ");
        Serial.println(command);

        // Handle commands for motor movement
        if (command == "F_SLOW") {
            moveForwardSlow();  // Modified to use the slower movement function
        } else if (command == "STOP") {
            stopMovement();
        }

        // Handle LED commands
        else if (command == "LED_WAKE") {
            digitalWrite(ledPin, HIGH);
            ledWakeActive = true;  // Activate wake LED state
            ledTimer = millis();   // Reset timer for LED wake state
            Serial.println("LED turned ON for wake indication");
        } else if (command == "LED_OFF") {
            digitalWrite(ledPin, LOW);
            isBlinking = false;    // Stop any blinking
            ledWakeActive = false; // Reset wake state
            Serial.println("LED turned OFF");
        } else if (command == "LED_BLINK") {
            isBlinking = true;     // Set flag for blinking
            Serial.println("LED started blinking");
        }
    }

    // Blink the LED if the isBlinking flag is set
    if (isBlinking) {
        if (millis() - ledTimer >= blinkInterval) {
            ledTimer = millis();  // Update the timer
            ledState = !ledState; // Toggle LED state
            digitalWrite(ledPin, ledState ? HIGH : LOW);
        }
    }

    // If LED wake state is active, turn it off after 5 seconds
    if (ledWakeActive && (millis() - ledTimer >= 5000)) {
        digitalWrite(ledPin, LOW); // Turn off LED after 5 seconds
        ledWakeActive = false;     // Deactivate wake state
        Serial.println("LED turned OFF after wake indication");
    }

    // Continuously read and send ultrasonic sensor data
    long distance = getDistance();
    Serial.println(distance); // Send only the distance value to Raspberry Pi
    delay(500);               // Short delay between readings
}

// Function to move both servos forward at a slower speed
void moveForwardSlow() {
    leftServo.write(95);  // Slow speed forward for left servo
    rightServo.write(85);  // Slow speed forward in opposite direction for right servo
    Serial.println("Moving Forward Slowly...");
}

// Function to stop both servos
void stopMovement() {
    leftServo.write(90);   // Stop (neutral position)
    rightServo.write(90);  // Stop (neutral position)
    Serial.println("Stopping...");
}

// Function to read distance from ultrasonic sensor
long getDistance() {
    digitalWrite(trigPin, LOW);  // Ensure trigPin is LOW
    delayMicroseconds(2);        // Wait for a short time
    digitalWrite(trigPin, HIGH); // Set trigPin HIGH
    delayMicroseconds(10);       // Wait for 10 microseconds
    digitalWrite(trigPin, LOW);  // Set trigPin LOW

    long duration = pulseIn(echoPin, HIGH); // Read the echo pin
    long distance = duration * 0.034 / 2;   // Calculate distance in cm
    return distance;
}