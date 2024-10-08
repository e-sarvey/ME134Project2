#include <WiFi.h>  // Internet connection baby!
#include <PubSubClient.h> // MQTT client subscribe and publish library for ESP
#include <AccelStepper.h> // Simplifies advanced stepper control including dual steppers in sync
#include <MultiStepper.h> // Sub class of Accel Stepper for running multiple steppers
#include <math.h> // so we can calculate things of course
#include <mysecrets.h> // git ignored file with IP address for MQTT and WIFI credentials

// Define pins for the two steppers
#define STEPPER1_STEP_PIN 4
#define STEPPER1_DIR_PIN 0
#define STEPPER2_STEP_PIN 12
#define STEPPER2_DIR_PIN 14

// Define LED pins for debugging. Each turns on depending on section of code running or completed.
#define WIFI_LED_PIN 35 
#define MQTT_LED_PIN 34
#define TRAJECTORY_LED_PIN 39
#define ERROR_LED_PIN 36

int stepsPerRevolution = 200; // to calculate angles

// Create AccelStepper objects for each motor
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// WIFI parameters (imported from mysecrets.h)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PSWD;

// MQTT Broker parameters (IP imported from mysecrets.h)
const char* mqtt_broker = MQTT_IP; 
const char* topic = "ME134/motors";
const int mqtt_port = 1883;

// WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  // Connect the ESP to the wifi network specified by ssid and password
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  
  Serial.println("Connected to Wi-Fi");
  digitalWrite(WIFI_LED_PIN, HIGH);  // Turn on Wi-Fi LED when connected
}

void moveMotors(float** angles, int numAngles) {
  // Move both stepper motors through an array of angles
  long targetPosition1;
  long targetPosition2;
  
  for (int currentAngleIndex = 0; currentAngleIndex < numAngles; currentAngleIndex++) {
    // Convert angles to steps
    targetPosition1 = lround(angles[0][currentAngleIndex] * stepsPerRevolution / 360.0);  // conversion from angles to steps
    targetPosition2 = lround(angles[1][currentAngleIndex] * stepsPerRevolution / 360.0);

    // Debugging: Print target positions in steps
    Serial.printf("Stepper 1 Target Position (steps): %ld\n", targetPosition1);
    Serial.printf("Stepper 2 Target Position (steps): %ld\n", targetPosition2);

    // Set the target positions
    stepper1.moveTo(targetPosition1);
    stepper2.moveTo(targetPosition2);

    // Use the blocking `runToPosition` method
    stepper1.runToPosition();  // Block until Stepper 1 reaches target
    stepper2.runToPosition();  // Block until Stepper 2 reaches target

    delay(10);  // Give some time for the motors to settle
  }

  digitalWrite(TRAJECTORY_LED_PIN, HIGH);  // Turn on Trajectory Finished LED
}

void parseAndMoveMotors(char* message, unsigned int length) {
  // Parse the message and create a 2xN angle array, then call moveMotors()
  int numAngles = 0;

  // Make a copy of the message
  char* messageCopy = new char[length + 1];
  strncpy(messageCopy, message, length);
  messageCopy[length] = '\0';

  // Count semicolons to determine the number of angle pairs
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

  // Tokenize and parse angle pairs
  char* pairToken = strtok(messageCopy, ";");
  int index = 0;

  while (pairToken != NULL && index < numAngles) {
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
    Serial.printf("%f", angles[0][i]);
    if (i < numAngles - 1) Serial.print(", ");
  }
  Serial.println();

  Serial.print("Parsed Motor 2 Angles: ");
  for (int i = 0; i < numAngles; i++) {
    Serial.printf("%f", angles[1][i]);
    if (i < numAngles - 1) Serial.print(", ");
  }
  Serial.println();

  moveMotors(angles, numAngles);

  // Clean up
  delete[] angles[0];
  delete[] angles[1];
  delete[] angles;
  delete[] messageCopy;
}

void MessageRecieved(char* topic, byte* message, unsigned int length) {
  // MQTT callback to receive messages from the broker
  Serial.printf("Message received in topic: %s\n", topic);
  
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)message[i];
  }
  msg[length] = '\0';  

  Serial.printf("Message: %s\n", msg);

  parseAndMoveMotors(msg, length);
  digitalWrite(MQTT_LED_PIN, HIGH);  // Turn on MQTT LED
}

void reconnect() {
  // Handle dropped WiFi connections and reconnect ESP to the internet and MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP32_Client")) {
      Serial.println("Connected");
      client.subscribe(topic);
    } else {
      Serial.printf("Failed, rc=%d, trying again in 5 seconds\n", client.state());
      digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on Error LED if initial connection fails
      delay(5000);
    }
  }
  digitalWrite(ERROR_LED_PIN, LOW);  // Turn off Error LED on successful connection
}

// MAIN SETUP CODE //
void setup() {
  Serial.begin(115200); // establish serial connection

  // Setup LEDs
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(MQTT_LED_PIN, OUTPUT);
  pinMode(TRAJECTORY_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  
  setup_wifi();
  
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(MessageRecieved);

  stepper1.setMaxSpeed(400); 
  stepper1.setAcceleration(300);
  
  stepper2.setMaxSpeed(400);
  stepper2.setAcceleration(300);

  // Set current position to 50 steps for both motors
  stepper1.setCurrentPosition(50);
  stepper2.setCurrentPosition(50);

  delay(10);
}

// MAIN LOOPING CODE //
void loop() {
  // Check WiFi connection, the callback handles the rest
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  
}
