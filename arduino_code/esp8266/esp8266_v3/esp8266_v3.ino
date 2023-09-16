#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <WebSocketsClient.h>

WebSocketsClient webSocket;
ESP8266WiFiMulti WiFiMulti;
SoftwareSerial receiverSerial(D1, D2);
SoftwareSerial senderSerial(D5, D6);

const char *ssid = "American Study";
const char *password = "66668888";
const char *ssid2 = "AmericanStudy";
const char *password2 = "66668888";
const char *ssid3 = "WiFi Poop";
const char *password3 = "CircularShit";

const char *serverIP = "192.168.1.134";
// const char* serverAddress = /*"ws://your-server-url"*/ "ws://localhost:" + serverPort + "/ws";
const int serverPort = 4000;
const int arraySize = 16;
float array[arraySize];

void setupWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(ssid, password);
  WiFiMulti.addAP(ssid2, password2);
  WiFiMulti.addAP(ssid3, password3);

  while (WiFiMulti.run() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("[SETUP] Connecting to WiFi...");
  }

  Serial.print("[SETUP] WiFi Connected. IP: ");
  Serial.println(WiFi.localIP());
}

void setupWebSocket()
{
  Serial.println("[SETUP] Initializing WebSocket...");
  webSocket.begin(serverIP, 4000, "/ws"); // Replace with your server IP and port
  // webSocket.begin(serverAddress, serverPort);  // Replace with your server IP and port
  webSocket.onEvent(webSocketEvent);
}

void setup()
{
  Serial.begin(9600);
  receiverSerial.begin(9600);
  senderSerial.begin(9600);
  Serial.println("\n[SETUP] BOOT");
  setupWiFi();
  setupWebSocket();
}

void loop()
{
  webSocket.loop();
  if (receiverSerial.available())
  {
    receiveDataAndSendToServer();
  }
}

void sendDataToServer(String data)
{
  webSocket.sendTXT(data);
}

void receiveDataAndSendToServer()
{
  // int i = 0;
  String data = receiverSerial.readStringUntil('\n'); // Read the JSON string from Serial
  Serial.println(data);
  // Check if the received data starts with "Sensors_data:"
  if (data.startsWith("Sensors_data:"))
  {
    // Extract the JSON part from the input string
    String jsonData = inputString.substring(data.indexOf('{'));
    Serial.print(jsonData);
    sendDataToServer(jsonData);
  }
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("WebSocket disconnected");
    break;
  case WStype_CONNECTED:
    Serial.println("WebSocket connected");
    break;
  case WStype_TEXT:
    Serial.print("Received message: ");
    Serial.println((char *)payload);
    String data = (char *)payload;
    DynamicJsonDocument doc(200); // Adjust buffer size as needed
    deserializeJson(doc, payload);
    String event_type = doc["type"];
    Serial.println(data);
    if (event_type.indexOf("control") != -1)
    {
      senderSerial.println(data);
    }
    break;
  }
}