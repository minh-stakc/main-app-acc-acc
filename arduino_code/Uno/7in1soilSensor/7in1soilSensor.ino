#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// RO to pin 8 & DI to pin 9 when using AltSoftSerial
#define RE 6
#define DE 7

const byte temp[] = {0x01, 0x03, 0x00, 0x13, 0x00, 0x01, 0x75, 0xcf}; //
const byte mois[] = {0x01, 0x03, 0x00, 0x12, 0x00, 0x01, 0x24, 0x0F};
const byte econ[] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xce};
const byte ph[] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0b}; // 0x0B64

const byte nitro[] = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x01, 0xE4, 0x0C};
const byte phos[] = {0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};

byte values[11];
AltSoftSerial mod;
SoftwareSerial mySerial(2, 3);

float soil_ph = 0.0, soil_mois = 0.0, soil_temp = 0.0, ec = 0.0, N = 0.0, P = 0.0, K = 0.0;
byte val1 = 0, val2 = 0, val3 = 0, val4 = 0, val5 = 0, val6 = 0, val7 = 0;

void setup()
{
  Serial.begin(9600);
  mod.begin(9600);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  // put RS-485 into receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  delay(3000);
}

void loop()
{
  if (mySerial.available())
  {
    readDataAndSendToMega();
  }
}

void readDataAndSendToMega()
{        
  String data = mySerial.readStringUntil('\n');
  Serial.println(data);
  if (data == "start")
  {
    val1 = moisture();
    soil_mois = val1 / 2.55;
    delay(1000);
    val2 = temperature();
    // soil_temp = val2/2.125-40;
    soil_temp = val2 / 10;
    delay(1000);
    val3 = econduc();
    ec = val3 * 78.43137254901961;
    delay(1000);
    val4 = phydrogen();
    // soil_ph = val4/42.5+3;
    soil_ph = val4 / 25 + 4;
    delay(1000);
    val5 = nitrogen();
    // N = val5*7.835294117647059;
    N = val5;
    delay(1000);
    val6 = phosphorous();
    // P = val6*7.835294117647059;
    P = val6;
    delay(1000);
    val7 = potassium();
    // K = val7*7.835294117647059;
    K = val7;
    delay(1000);
    Serial.print("Moisture: ");
    Serial.print(soil_mois);
    Serial.println(" %");
    delay(1000);
    Serial.print("Temperature: ");
    Serial.print(soil_temp);
    Serial.println(" C");
    delay(1000);
    Serial.print("EC: ");
    Serial.print(ec);
    Serial.println(" us/cm");
    delay(1000);
    Serial.print("ph: ");
    Serial.print(soil_ph);
    Serial.println(" ph");
    delay(1000);
    Serial.print("Nitrogen: ");
    Serial.print(N);
    Serial.println(" mg/kg");
    delay(1000);
    Serial.print("Phosphorous: ");
    Serial.print(P);
    Serial.println(" mg/kg");
    delay(1000);
    Serial.print("Potassium: ");
    Serial.print(K);
    Serial.println(" mg/kg");
    Serial.println();

    StaticJsonDocument<200> jsonDoc;
    jsonDoc["soil_mois"] = soil_mois;
    jsonDoc["soil_temp"] = soil_temp;
    jsonDoc["ec"] = ec;
    jsonDoc["soil_ph"] = soil_ph;
    jsonDoc["nitrogen"] = N;
    jsonDoc["phosphorous"] = P;
    jsonDoc["potassium"] = K;
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    Serial.print("Sensors_data: " + jsonString + "\n");
    mySerial.print("Sensors_data: " + jsonString + "\n");

    delay(3000);
  }
}

byte moisture()
{
  // clear the receive buffer
  mod.flushInput();

  // switch RS-485 to transmit mode
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);

  // write out the message
  for (uint8_t i = 0; i < sizeof(mois); i++)
    mod.write(mois[i]);

  // wait for the transmission to complete
  mod.flush();

  // switching RS485 to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  // delay to allow response bytes to be received!
  delay(200);

  // read in the received bytes
  for (byte i = 0; i < 7; i++)
  {
    values[i] = mod.read();
    // Serial.print(values[i], HEX);
    // Serial.print(' ');
  }
  return values[4];
}

byte temperature()
{
  // clear the receive buffer
  mod.flushInput();

  // switch RS-485 to transmit mode
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);

  // write out the message
  for (uint8_t i = 0; i < sizeof(temp); i++)
    mod.write(temp[i]);

  // wait for the transmission to complete
  mod.flush();

  // switching RS485 to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  // delay to allow response bytes to be received!
  delay(200);

  // read in the received bytes
  for (byte i = 0; i < 7; i++)
  {
    values[i] = mod.read();
    // Serial.print(values[i], HEX);
    // Serial.print(' ');
  }
  return values[3] << 8 | values[4];
}

byte econduc()
{
  // clear the receive buffer
  mod.flushInput();

  // switch RS-485 to transmit mode
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);

  // write out the message
  for (uint8_t i = 0; i < sizeof(econ); i++)
    mod.write(econ[i]);

  // wait for the transmission to complete
  mod.flush();

  // switching RS485 to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  // delay to allow response bytes to be received!
  delay(200);

  // read in the received bytes
  for (byte i = 0; i < 7; i++)
  {
    values[i] = mod.read();
    // Serial.print(values[i], HEX);
    // Serial.print(' ');
  }
  return values[4];
}

byte phydrogen()
{
  // clear the receive buffer
  mod.flushInput();
  // switch RS-485 to transmit mode
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);

  // write out the message
  for (uint8_t i = 0; i < sizeof(ph); i++)
    mod.write(ph[i]);

  // wait for the transmission to complete
  mod.flush();

  // switching RS485 to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  // delay to allow response bytes to be received!
  delay(200);

  // read in the received bytes
  for (byte i = 0; i < 7; i++)
  {
    values[i] = mod.read();
    // Serial.print(values[i], HEX);
    // Serial.print(' ');
  }
  return values[4];
}

byte nitrogen()
{
  // clear the receive buffer
  mod.flushInput();

  // switch RS-485 to transmit mode
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);

  // write out the message
  for (uint8_t i = 0; i < sizeof(nitro); i++)
    mod.write(nitro[i]);

  // wait for the transmission to complete
  mod.flush();

  // switching RS485 to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  // delay to allow response bytes to be received!
  delay(200);

  // read in the received bytes
  for (byte i = 0; i < 7; i++)
  {
    values[i] = mod.read();
    // Serial.print(values[i], HEX);
    // Serial.print(' ');
  }
  return values[4];
}

byte phosphorous()
{
  mod.flushInput();
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);
  for (uint8_t i = 0; i < sizeof(phos); i++)
    mod.write(phos[i]);
  mod.flush();
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  // delay to allow response bytes to be received!
  delay(200);
  for (byte i = 0; i < 7; i++)
  {
    values[i] = mod.read();
    // Serial.print(values[i], HEX);
    // Serial.print(' ');
  }
  return values[4];
}

byte potassium()
{
  mod.flushInput();
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);
  for (uint8_t i = 0; i < sizeof(pota); i++)
    mod.write(pota[i]);
  mod.flush();
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  // delay to allow response bytes to be received!
  delay(200);
  for (byte i = 0; i < 7; i++)
  {
    values[i] = mod.read();
    // Serial.print(values[i], HEX);
    // Serial.print(' ');
  }
  return values[4];
}