#include <WiFi.h>  
#include <PubSubClient.h> // MQTT library for Arduino
#include <AccelStepper.h> // Library made by Mike Mcauley for advanced control of steppers
#include <MultiStepper.h> // Class from AccelStepper for controlling multiple steppers simultaneously
#include <math.h>

// Define pins for the two steppers (change these based on wiring)
#define STEPPER1_STEP_PIN 36
#define STEPPER1_DIR_PIN 39
#define STEPPER2_STEP_PIN 34
#define STEPPER2_DIR_PIN 35

int stepsPerRevolution = 200;  // 200 steps for a standard NEMA 17 stepper motor

// Create AccelStepper objects for each motor
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// Create MultiStepper object to manage multiple steppers simultaneously
MultiStepper steppers;

// Replace with your network credentials
const char* ssid = "Tufts_Robot";
const char* password = "";

// MQTT Broker details
const char* mqtt_broker = ""; // my IP address. If MQTT is giving issues, use ifconfig to check this
const char* topic = "ME134/motor";  
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
}

// Function to move motors through an array of angles
void moveMotors(float** angles, int numAngles) {
  long targetPositions[2];

  // Iterate through the array of angles
  for (int currentAngleIndex = 0; currentAngleIndex < numAngles; currentAngleIndex++) {
    // Convert angles to steps for each motor and store in the targetPositions array
    targetPositions[0] = lround(angles[0][currentAngleIndex] * stepsPerRevolution / 360.0);  // Stepper 1
    targetPositions[1] = lround(angles[1][currentAngleIndex] * stepsPerRevolution / 360.0);  // Stepper 2

    // Move steppers to the target positions
    steppers.moveTo(targetPositions);

    // Use steppers.run() to manage motor movements until both reach their target
    while (steppers.run()) {
      // Continue running until the movement is completed "Pass" here in Python lol.
      // Notably, this method is non-blocking AND indicates end of motor movement.
    }
    delay(10);  // take a short pause before moving to next set of points
  }
}

// Function to parse the MQTT message into angles
void parseAndMoveMotors(char* message, unsigned int length) {
  // Create dynamic arrays for angles
  int numAngles = length / 4;  // Assuming each angle is 4 bytes (e.g., "15.0,30.0")
  float** angles = new float*[2];
  angles[0] = new float[numAngles];  // For Stepper 1 angles
  angles[1] = new float[numAngles];  // For Stepper 2 angles
  
  // Parse the message into two arrays of angles
  char* token = strtok(message, ",");
  int index = 0;
  
  while (token != NULL && index < numAngles) {
    angles[0][index] = atof(token);  // Parse angle for Stepper 1
    token = strtok(NULL, ",");
    angles[1][index] = atof(token);  // Parse angle for Stepper 2
    token = strtok(NULL, ",");
    index++;
  }

  // Move motors using the parsed angles
  moveMotors(angles, numAngles);

  // Free dynamically allocated memory
  delete[] angles[0];
  delete[] angles[1];
  delete[] angles;
}

// Callback function to handle incoming messages
void MessageRecieved(char* topic, byte* message, unsigned int length) {
  Serial.print("Message received in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  
  // Convert MQTT message to a string
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)message[i];
  }
  msg[length] = '\0';  // Null-terminate the string

  Serial.print("Message: ");
  Serial.println(msg);

  // Parse and move the motors based on the received message
  parseAndMoveMotors(msg, length);
}

// Reconnect function to ensure the client stays connected
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
      delay(5000);
    }
  }
}

// MAIN SETUP CODE //
void setup() {
  Serial.begin(115200); // establish serial connection
  setup_wifi(); 
  
  // Setup MQTT server and callback
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(MessageRecieved);

  // STEPPER SETUP
  // Set max speed and acceleration for each stepper (adjust these values based on your motors)
  stepper1.setMaxSpeed(200); 
  stepper1.setAcceleration(200);
  
  stepper2.setMaxSpeed(200);
  stepper2.setAcceleration(200);

  // Add the steppers to the MultiStepper manager
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

}

// MAIN LOOPING CODE: "While True" //
void loop() {
  // make sure MQTT/wifi connection is maintained and reconnected if dropped during execution
  if (!client.connected()) {
    reconnect(); // reconnecting also re-subscribes to MQTT topics!
  }
  client.loop();  // Keep checking for incoming MQTT messages
}
