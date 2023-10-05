#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <WebSocketsClient.h>

WebSocketsClient webSocket;
ESP8266WiFiMulti WiFiMulti;
// SoftwareSerial senderSerial(D1, D2);
SoftwareSerial senderSerial(D5, D6);

// const char *ssid = "American Study";
// const char *password = "66668888";
// const char *ssid2 = "AmericanStudy T1";
// const char *password2 = "66668888";
// const char *ssid3 = "WiFi Poop";
// const char *password3 = "CircularShit";

const int numOfWifis = 4;
const char *ssids[numOfWifis] = {"HB4", "American Study", "AmericanStudy T1", "WiFi Poop"};
const char *passwords[numOfWifis] = {"oanhslhk", "66668888", "66668888", "CircularShit"};

//testing with local-host
// const char *serverAddress = "192.168.0.103";
// const int serverPort = 4000;

const char *serverAddress = "automaticcropcaretaker.com";
const int serverPort = 80;

const char *serverURL = "/ws";
const char *serverProtocol = "arduino";

bool needBooting = true;


void setupWiFi() {
  WiFi.mode(WIFI_STA);
  // WiFiMulti.addAP(ssid, password);
  // WiFiMulti.addAP(ssid2, password2);
  // WiFiMulti.addAP(ssid3, password3);
  int i = 0;
  for(i = 0; i < numOfWifis; i++){
    WiFiMulti.addAP(ssids[i], passwords[i]);
  }

  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(1000);
    Serial.println("[SETUP] Connecting to WiFi...");
  }

  Serial.print("[SETUP] WiFi Connected. IP: ");
  Serial.println(WiFi.localIP());
}

void setupWebSocket() {
  Serial.println("[SETUP] Initializing WebSocket...");
  webSocket.begin(serverAddress, serverPort, serverURL, serverProtocol);
  webSocket.onEvent(webSocketEvent);
}

void setup() {
  delay(3000);
  Serial.begin(9600);
  senderSerial.begin(9600);
}

void loop() {
  if (needBooting == true) {
    Serial.println("\n[SETUP] BOOT");
    setupWiFi();
    setupWebSocket();
    needBooting = false;
  }
  webSocket.loop();
  if (senderSerial.available()) {
    receiveDataAndSendToServer();
  }
}

void sendDataToServer(String data) {
  webSocket.sendTXT(data);
}

void receiveDataAndSendToServer() {
  String data = senderSerial.readStringUntil('\n');  // Read the JSON string from Serial
  // Serial.println(data);
  // Check if the received data starts with "Sensors_data:"
  if (data.startsWith("[MEGA] Sensors_data:")) {
    // Extract the JSON part from the input string
    String jsonData = data.substring(data.indexOf('{'));
    Serial.print(jsonData);
    sendDataToServer(jsonData);
  }
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WSc] Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("[WSc] Connected");
      sendDataToServer((String) "{\"type\":\"connection\",\"data\":\"ESP8266\"}");
      break;
    case WStype_TEXT:
      Serial.print("[WSc] Received message:");
      String data = (char *)payload;
      Serial.println(data);
      // DynamicJsonDocument doc(64);  // Adjust buffer size as needed
      // deserializeJson(doc, data);
      // String eventType = doc["type"];

      // Check if received message is a ping
      //"{"type":"control/both_motors"}" <- This is sample data from server
      if (data.indexOf("\"type\":\"ping\"") != -1) {
        // If it's a ping, send a pong back
        webSocket.sendTXT("{\"type\":\"pong\",\"from\":\"ESP8266\"}");
      }else {
        senderSerial.println(data);
      }
      break;
  }
}