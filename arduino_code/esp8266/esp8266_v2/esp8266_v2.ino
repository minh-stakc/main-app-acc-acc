#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <WebSocketsClient.h>

WebSocketsClient webSocket;
ESP8266WiFiMulti WiFiMulti;
SoftwareSerial receiverSerial(D1, D2);
SoftwareSerial senderSerial(D5, D6);

const char* ssid = "American Study";
const char* password = "66668888";
const char* ssid2 = "AmericanStudy T1";
const char* password2 = "66668888";
const char* ssid3 = "WiFi Poop";
const char* password3 = "CircularShit";

<<<<<<< HEAD:arduino_code/esp8266_v2/esp8266_v2.ino
const char* serverIP = "192.168.100.83";
=======
const char* serverIP = "192.168.1.134";
>>>>>>> ab5333a3257739977389f2b46491248fc86afb08:arduino_code/esp8266/esp8266_v2/esp8266_v2.ino
// const char* serverAddress = /*"ws://your-server-url"*/ "ws://localhost:" + serverPort + "/ws";
const int serverPort = 4000;
const int arraySize = 16;
float array[arraySize];


void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(ssid, password);
  WiFiMulti.addAP(ssid2, password2);
  WiFiMulti.addAP(ssid3, password3);

  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(1000);
    Serial.println("[SETUP] Connecting to WiFi...");
  }

  Serial.print("[SETUP] WiFi Connected. IP: ");
  Serial.println(WiFi.localIP());
}

void setupWebSocket() {
  Serial.println("[SETUP] Initializing WebSocket...");
<<<<<<< HEAD:arduino_code/esp8266_v2/esp8266_v2.ino
  webSocket.begin("192.168.1.118", 4000, "/ws"); // Replace with your server IP and port
=======
  webSocket.begin(serverIP, 4000, "/ws"); // Replace with your server IP and port
>>>>>>> ab5333a3257739977389f2b46491248fc86afb08:arduino_code/esp8266/esp8266_v2/esp8266_v2.ino
  // webSocket.begin("ws://localhost:81/ws", serverPort);  // Replace with your server IP and port
  webSocket.onEvent(webSocketEvent);
}



void setup() {
  Serial.begin(115200);
  receiverSerial.begin(9600);
  senderSerial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println("[SETUP] BOOT");
  setupWiFi();
  setupWebSocket();
}

void loop() {
  webSocket.loop();
  if (receiverSerial.available()) {
    receiveDataAndSendToServer();
  }
}

void sendDataToServer(String data) {
  webSocket.sendTXT(data);
}

void receiveDataAndSendToServer() {
  int i = 0;
  String data = receiverSerial.readStringUntil('\n');  // Read the JSON string from Serial
  Serial.println(data);
  char charArray[data.length() + 1];
  data.toCharArray(charArray, sizeof(charArray));

  // Initialize strtok_r
  char* token;
  char* rest = charArray;
  // Parse and extract float values
  token = strtok_r(rest, " ", &rest);
  String token_str = token;
  if (token_str.indexOf("Sensors_data:") != -1) {
  while ((token = strtok_r(rest, " ", &rest))) {
      if (token != NULL) {
        float floatValue = strtod(token, NULL);
        array[i] = floatValue;

        if (i == arraySize) {
          i = 0;
          break;
        }
        i++;
      }
    }
  }

  DynamicJsonDocument jsonDoc(512);  // Adjust the size as needed
  jsonDoc["type"] = "sensors_data";
  jsonDoc["PH"] = array[0];
  jsonDoc["TDS"] = array[1];
  jsonDoc["Lux"] = array[2];
  jsonDoc["Humidity"] = array[3];
  jsonDoc["Temperature"] = array[4];
  jsonDoc["Heat Index"] = array[5];
  jsonDoc["Corrected_PPM"] = array[6];
  jsonDoc["Dust Density"] = array[7];
  jsonDoc["Running average"] = array[8];
  // jsonDoc["Nitrogen"] = array[9];
  // jsonDoc["Phosphorus"] = array[10];
  // jsonDoc["Potassium"] = array[11];
  // jsonDoc["pHsoil"] = array[12];
  // jsonDoc["EC"] = array[13];
  // jsonDoc["Tempsoil"] = array[14];
  // jsonDoc["Humsoil"] = array[15];

  // Serialize JSON to a string
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  sendDataToServer(jsonString);
  Serial.print(jsonString);
  // Send JSON to Firebase
  // String path = "/sensor_data.json";  // Change this path as needed
  // if (Firebase.setString(path, jsonString)) {
  //   Serial.println("Data sent to Firebase successfully");
  // } else {
  //   Serial.println("Failed to send data to Firebase");
  // }
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      break;
    case WStype_TEXT:
      Serial.print("Received message: ");
      Serial.println((char*)payload);
      String data = (char*)payload;
      DynamicJsonDocument doc(512); // Adjust buffer size as needed
      deserializeJson(doc, payload);
      String event_type = doc["type"];
      Serial.println(data);
      if(event_type.indexOf("control") != -1){
        senderSerial.println(data);
      }
      // Check the event type and take appropriate action
      // Handle the event based on the message
      // Process the received message and change NeoPixel color if needed
      break;
  }
}