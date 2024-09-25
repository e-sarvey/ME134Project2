#include <WiFi.h>  
#include <PubSubClient.h> // MQTT library for Arduino

// Replace with your network credentials
const char* ssid = "Tufts_Robot";
const char* password = "";

// MQTT Broker details
const char* mqtt_broker = "10.243.82.33"; // Replace with your broker's IP address
const char* topic = "test/topic";  // Replace with your desired topic
const int mqtt_port = 1883;

// WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to Wi-Fi
void setup_wifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  
  Serial.println("Connected to Wi-Fi");
}

// Callback function to handle incoming messages
void MessageReceived(char* topic, byte* message, unsigned int length) {
  Serial.print("Message received in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  }
  Serial.println();
}

// Reconnect function to ensure the client stays connected
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP32_Client")) {
      Serial.println("Connected");
      client.subscribe(topic); // Subscribe to the test topic
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200); // Establish serial connection
  setup_wifi(); 
  
  // Setup MQTT server and callback
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(MessageReceived);
}

void loop() {
  // Ensure MQTT/wifi connection is maintained and reconnected if dropped during execution
  if (!client.connected()) {
    reconnect(); // Reconnect also re-subscribes to MQTT topics!
  }
  client.loop();

  // Publish a test message every 5 seconds
  static unsigned long lastPublish = 0;
  if (millis() - lastPublish > 5000) {
    String message = "Hello, MQTT!";
    client.publish(topic, message.c_str());
    Serial.print("Published message: ");
    Serial.println(message);
    lastPublish = millis();
  }
}
