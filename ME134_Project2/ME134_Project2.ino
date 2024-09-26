#include <WiFi.h>  
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include <mysecrets.h>
// Define pins for the two steppers
#define STEPPER1_STEP_PIN 4
#define STEPPER1_DIR_PIN 0
#define STEPPER2_STEP_PIN 12
#define STEPPER2_DIR_PIN 14

// Define LED pins
#define WIFI_LED_PIN 35  // Change to your LED pin
#define MQTT_LED_PIN 34  // Change to your LED pin
#define TRAJECTORY_LED_PIN 39  // Change to your LED pin
#define ERROR_LED_PIN 36  // Change to your LED pin

/// Define the initial offset for the steppers (in degrees)
const float INITIAL_OFFSET_1 = -90.0;  // Offset for Stepper 1
const float INITIAL_OFFSET_2 = -90.0;  // Offset for Stepper 2

int stepsPerRevolution = 200;

// Create AccelStepper objects for each motor
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// Create MultiStepper object
MultiStepper steppers;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PSWD;

// MQTT Broker details
const char* mqtt_broker = MQTT_IP; 
const char* topic = "ME134/motors";
const int mqtt_port = 1883;

// WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  
  Serial.println("Connected to Wi-Fi");
  digitalWrite(WIFI_LED_PIN, HIGH);  // Turn on Wi-Fi LED
}


// Function to move motors through an array of angles
void moveMotors(float** angles, int numAngles) {
  long targetPositions[2];

  for (int currentAngleIndex = 0; currentAngleIndex < numAngles; currentAngleIndex++) {
    // Apply the offset to the angles
    float adjustedAngle1 = angles[0][currentAngleIndex] + INITIAL_OFFSET_1;
    float adjustedAngle2 = angles[1][currentAngleIndex] + INITIAL_OFFSET_2;

    targetPositions[0] = lround(adjustedAngle1 * stepsPerRevolution / 360.0);  
    targetPositions[1] = lround(adjustedAngle2 * stepsPerRevolution / 360.0);  
    Serial.print("Stepper 1 Target Position: ");
    Serial.println(targetPositions[0]);
    Serial.print("Stepper 2 Target Position: ");
    Serial.println(targetPositions[1]);
    
    steppers.moveTo(targetPositions);  // Move both motors simultaneously
    steppers.runSpeedToPosition();     // Blocking function, waits until both motors reach target

    delay(2000);  // Small pause between movements
  }
  digitalWrite(TRAJECTORY_LED_PIN, HIGH);  // Turn on Trajectory LED
}



// Function to parse the MQTT message into angles
// Function to parse the MQTT message into angles
void parseAndMoveMotors(char* message, unsigned int length) {
  int numAngles = 0;

  // Make a copy of the message to avoid modifying the original
  char* messageCopy = new char[length + 1];
  strncpy(messageCopy, message, length);
  messageCopy[length] = '\0';

  // Count the number of semicolons to determine how many angle pairs there are
  for (int i = 0; i < length; i++) {
    if (message[i] == ';') {
      numAngles++;
    }
  }
  numAngles++;  // One more than the number of semicolons

  // Allocate memory for angles
  float** angles = new float*[2];
  angles[0] = new float[numAngles];  
  angles[1] = new float[numAngles];  

  // Use a single strtok loop to tokenize and parse the angle pairs
  char* pairToken = strtok(messageCopy, ";");
  int index = 0;

  while (pairToken != NULL && index < numAngles) {
    // Use sscanf to parse both angles from the pair at once
    float angle1, angle2;
    if (sscanf(pairToken, "%f,%f", &angle1, &angle2) == 2) {
      angles[0][index] = angle1;
      angles[1][index] = angle2;
    } else {
      Serial.println("Error parsing angles!");
    }

    pairToken = strtok(NULL, ";");
    index++;
  }

  // Debugging: Print parsed angles
  Serial.print("Parsed Motor 1 Angles: ");
  for (int i = 0; i < numAngles; i++) {
    Serial.print(angles[0][i]);
    if (i < numAngles - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();

  Serial.print("Parsed Motor 2 Angles: ");
  for (int i = 0; i < numAngles; i++) {
    Serial.print(angles[1][i]);
    if (i < numAngles - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();

  Serial.print("Total Parsed Angle Pairs: ");
  Serial.println(numAngles);

  moveMotors(angles, numAngles);

  // Clean up
  delete[] angles[0];
  delete[] angles[1];
  delete[] angles;
  delete[] messageCopy;
}



// Callback function to handle incoming messages
void MessageRecieved(char* topic, byte* message, unsigned int length) {
  Serial.print("Message received in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)message[i];
  }
  msg[length] = '\0';  

  Serial.print("Message: ");
  Serial.println(msg);

  parseAndMoveMotors(msg, length);
  digitalWrite(MQTT_LED_PIN, HIGH);  // Turn on MQTT LED
}

// Reconnect function
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP32_Client")) {
      Serial.println("Connected");
      client.subscribe(topic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on Error LED
      delay(5000);
    }
  }
  digitalWrite(ERROR_LED_PIN, LOW);  // Turn off Error LED on successful connection
}

// MAIN SETUP CODE //
void setup() {
  Serial.begin(115200); // establish serial connection
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(MQTT_LED_PIN, OUTPUT);
  pinMode(TRAJECTORY_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  
  setup_wifi(); 
  
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(MessageRecieved);

  stepper1.setMaxSpeed(20); 
  stepper1.setAcceleration(10);
  
  stepper2.setMaxSpeed(20);
  stepper2.setAcceleration(10);

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
}

// MAIN LOOPING CODE: "While True" //
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  
}
