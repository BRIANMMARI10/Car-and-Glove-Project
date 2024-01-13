/******************************************************************************
Glove

https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/coding
https://learn.sparkfun.com/tutorials/flex-sensor-hookup-guide/all
https://randomnerdtutorials.com/esp32-esp-now-wi-fi-web-server/
******************************************************************************/

// Sensors
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ESP-NOW
#include <esp_now.h>
#include <WiFi.h>


/*---------------Sensors---------------*/
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

const int PALM_PIN = 35;
const int THUMB_PIN = 34;
const int INDEX_PIN = 39;
const int MIDDLE_PIN = 36;
const int VIBE_MOTOR_PIN = 32;

const int PALM_threshold = 3330; // larger is press
const int THUMB_threshold = 1900; // smaller is bent
const int INDEX_threshold = 2000; // smaller is bent
const int MIDDLE_threshold = 2100; // smaller is bent
const int LEFT_TURN_threshold = -3; // < -3 is left
const int RIGHT_TURN_threshold = 3; // > 3 is right


/*---------------ESP-NOW---------------*/
// ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0xB8, 0xD6, 0x1A, 0x5D, 0xD8, 0x78};

// Datatype of data to send
typedef struct test_struct {
  int x;
} test_struct;

// Initialize data
test_struct test;

// Store peer information
esp_now_peer_info_t peerInfo;

// Print if message is successfully sent or not for which MAC address
// Run this callback function when message is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


/*---------------Movements---------------*/
void turnLeft() {
  Serial.println("Turn left");
  digitalWrite(VIBE_MOTOR_PIN, HIGH);
  delay(5);
  digitalWrite(VIBE_MOTOR_PIN, LOW);

  // Assign value to each data attribute
  test.x = 1;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
  
  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void turnRight() {
  Serial.println("Turn right");
  digitalWrite(VIBE_MOTOR_PIN, HIGH);
  delay(5);
  digitalWrite(VIBE_MOTOR_PIN, LOW);

  // Assign value to each data attribute
  test.x = 2;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
  
  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void faceStraight() {
  Serial.println("Face straight");
  digitalWrite(VIBE_MOTOR_PIN, HIGH);
  delay(5);
  digitalWrite(VIBE_MOTOR_PIN, LOW);

  // Assign value to each data attribute
  test.x = 3;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
  
  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void goForward() {
  Serial.println("Go forward");
  digitalWrite(VIBE_MOTOR_PIN, HIGH);
  delay(5);
  digitalWrite(VIBE_MOTOR_PIN, LOW);

  // Assign value to each data attribute
  test.x = 4;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
  
  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void goBackward() {
  Serial.println("Go backward");
  digitalWrite(VIBE_MOTOR_PIN, HIGH);
  delay(5);
  digitalWrite(VIBE_MOTOR_PIN, LOW);

  // Assign value to each data attribute
  test.x = 5;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
  
  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void stop() {
  Serial.println("Stop");

  // Assign value to each data attribute
  test.x = 6;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
  
  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}


void setup() 
{
  Serial.begin(115200);

  /*---------------Sensors---------------*/
  /* Initialise the sensor */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }

  accel.setRange(LSM303_RANGE_4G);
  Serial.print("Range set to: ");
  lsm303_accel_range_t new_range = accel.getRange();
  switch (new_range) {
  case LSM303_RANGE_2G:
    Serial.println("+- 2G");
    break;
  case LSM303_RANGE_4G:
    Serial.println("+- 4G");
    break;
  case LSM303_RANGE_8G:
    Serial.println("+- 8G");
    break;
  case LSM303_RANGE_16G:
    Serial.println("+- 16G");
    break;
  }

  accel.setMode(LSM303_MODE_NORMAL);
  Serial.print("Mode set to: ");
  lsm303_accel_mode_t new_mode = accel.getMode();
  switch (new_mode) {
  case LSM303_MODE_NORMAL:
    Serial.println("Normal");
    break;
  case LSM303_MODE_LOW_POWER:
    Serial.println("Low Power");
    break;
  case LSM303_MODE_HIGH_RESOLUTION:
    Serial.println("High Resolution");
    break;
  }

  pinMode(PALM_PIN, INPUT);
  pinMode(THUMB_PIN, INPUT);
  pinMode(INDEX_PIN, INPUT);
  pinMode(MIDDLE_PIN, INPUT);
  pinMode(VIBE_MOTOR_PIN, OUTPUT);


  /*---------------ESP-NOW---------------*/
  // Set device as Wifi station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register OnDataSent() as callback function  
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}


void loop() 
{
  /*---------------Sensors---------------*/
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  /* Display the results (acceleration is measured in m/s^2) */
  //Serial.print("X: ");
  //Serial.print(event.acceleration.x);
  //Serial.print("  ");
  //Serial.print("Y: ");
  //Serial.print(event.acceleration.y);
  //Serial.print("  ");
  //Serial.print("Z: ");
  //Serial.print(event.acceleration.z);
  //Serial.print("  ");
  //Serial.println("m/s^2");
  int palmReading = analogRead(PALM_PIN);
  int thumbReading = analogRead(THUMB_PIN);
  int indexReading = analogRead(INDEX_PIN);
  int middleReading = analogRead(MIDDLE_PIN);
  //Serial.println("Palm Reading: " + String(palmReading));
  //Serial.println("Thumb Reading: " + String(thumbReading));
  //Serial.println("Index Reading: " + String(indexReading));
  //Serial.println("Middle Reading: " + String(middleReading));
  
  if (middleReading >= 2100) {
    faceStraight();
  } else if (thumbReading < 1900) {
    turnLeft();
  } else {
    turnRight();
  }
  
  
  /*else if (event.acceleration.z < LEFT_TURN_threshold) {
    turnLeft();
  } else if (event.acceleration.z > RIGHT_TURN_threshold) {
    turnRight();
  }*/


  if (palmReading < 3330) {
    stop();
  } else {
    if (indexReading >= 2000) {
      goForward();
    } else {
      goBackward();
    }
  }

  Serial.println("_________________________________________________");
  delay(500);
}
