/*
https://randomnerdtutorials.com/esp32-esp-now-wi-fi-web-server/
*/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include "Arduino.h"
#include <ESP32Servo.h>


// Servo connections
Servo myservo; // create servo object to control a servo
int pos = 0; // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int servoPin = 13;


// Motor A connections
int enA = 14;
int in1 = 27;
int in2 = 26;


// Structure example to receive data
// Must match the sender structure
typedef struct test_struct {
int x;
} test_struct;


// Initialize data
test_struct myData;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
memcpy(&myData, incomingData, sizeof(myData));
Serial.print("Bytes received: ");
Serial.println(len);
Serial.print("x: ");
Serial.println(myData.x);


if (myData.x == 1) turnLeft();
else if (myData.x == 2) turnRight();
else if (myData.x == 3) faceStraight();
else if (myData.x == 4) goForward();
else if (myData.x == 5) goBackward();
else if (myData.x == 6) stop();
}


void turnLeft() {
Serial.println("Turning left...");
myservo.write(0);
};


void turnRight() {
Serial.println("Turning right...");
myservo.write(180);
};


void faceStraight() {
Serial.println("Facing straight...");
myservo.write(90);
};


void goForward() {
Serial.println("Going forward...");
// Set motors to maximum speed
analogWrite(enA, 255);
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
};


void goBackward() {
Serial.println("Going backward...");
// Set motors to maximum speed
analogWrite(enA, 255);


digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
};


void stop() {
Serial.println("Stopping...");
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
};


void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);


  // Servo setup
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep


  // Motor setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);


  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.println("starts");

  // Register self as receiver
  //esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
void loop() {
}
