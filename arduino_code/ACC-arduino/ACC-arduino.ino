#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "MQ135.h"

#include <GP2YDustSensor.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>

#define VREF 5.0
#define SCOUNT 30
#define PH_PIN A0
#define TdsSensorPin A1
#define photodiodePin A2
#define PIN_MQ135 A3
#define SHARP_LED_PIN 32
#define SHARP_VO_PIN A4
#define DHTPIN 10
#define DHTTYPE DHT22

#define MLeftRL_EN 4   // băm xung
#define MLeftR_PWM 26  // điều khiển chiều quay
#define MLeftL_PWM 27  // điều khiển chiều quay

#define MRightRL_EN 5   // băm xung
#define MRightR_PWM 28  // điều khiển chiều quay
#define MRightL_PWM 29  // điều khiển chiều quay

#define CylRL_EN 2   // băm xung
#define CylR_PWM 22  // điều khiển chiều quay
#define CylL_PWM 23  // điều khiển chiều quay

#define PumpRL_EN 3   // băm xung
#define PumpR_PWM 24  // điều khiển chiều quay
#define PumpL_PWM 25  // điều khiển chiều quay

#define SLeftRL_EN 6
#define SLeftR_PWM 34  // điều khiển chiều quay
#define SLeftL_PWM 35  // điều khiển chiều quay

#define SRightRL_EN 7
#define SRightR_PWM 36  // điều khiển chiều quay
#define SRightL_PWM 37  // điều khiển chiều quay

#define SLeftUpButton 38    // to see if left servo has fully turned 90 deg
#define SLeftDownButton 39  // to see if left servo has fully turned 0 deg

#define SRightUpButton 40    // to see if right servo has fully turned 90 deg
#define SRightDownButton 41  // to see if eight servo has fully turned 0 deg

#define PIN_RELAY_LED_1 30
#define PIN_RELAY_LED_2 31

#define SSerialRX 11  // RS485 Serial Receive pin
#define SSerialTX 12  // RS485 Serial Transmit pin

SoftwareSerial mySerial(SSerialRX, SSerialTX);
ModbusMaster node;

float volt, phValue = 25;
DFRobot_PH water_ph;
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;

MQ135 mq135_sensor = MQ135(PIN_MQ135);
DHT dht = DHT(DHTPIN, DHTTYPE);
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1014AU0F, SHARP_LED_PIN, SHARP_VO_PIN);

static int left_servo_state, right_servo_state, servos_state, motors_state, pump_state, led_state, cylinder_state;  //= direction
static int left_motor_speed, right_motor_speed;

uint16_t data[6];
float ph;
float Temp, Humi;           //Sensor Temperarure and Humidity
float Nito, Photpho, Kali;  // Smart water sensor PH/ORP
float EC;
float result;

void setup() {
  // debug serial
  Serial.begin(9600);
  // sender serial
  Serial3.begin(9600);
  // arduino uno/npk value receiver serial
  // Serial2.begin(9600);
  // esp8266 commands receiver serial
  Serial3.begin(9600);

  mySerial.begin(9600);
  node.begin(1, mySerial);

  Serial.println("\n[SETUP] BOOT");

  pinMode(TdsSensorPin, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(PIN_MQ135, INPUT);
  pinMode(photodiodePin, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(SHARP_LED_PIN, OUTPUT);
  pinMode(SHARP_VO_PIN, INPUT);

  dht.begin();
  water_ph.begin();
  dustSensor.begin();

  pinMode(MLeftRL_EN, OUTPUT);
  pinMode(MLeftR_PWM, OUTPUT);
  pinMode(MLeftL_PWM, OUTPUT);

  pinMode(MRightRL_EN, OUTPUT);
  pinMode(MRightL_PWM, OUTPUT);
  pinMode(MRightR_PWM, OUTPUT);

  pinMode(SLeftRL_EN, OUTPUT);
  pinMode(SLeftR_PWM, OUTPUT);
  pinMode(SLeftL_PWM, OUTPUT);

  pinMode(SRightRL_EN, OUTPUT);
  pinMode(SRightL_PWM, OUTPUT);
  pinMode(SRightR_PWM, OUTPUT);

  pinMode(CylRL_EN, OUTPUT);
  pinMode(CylR_PWM, OUTPUT);
  pinMode(CylL_PWM, OUTPUT);

  pinMode(PumpRL_EN, OUTPUT);
  pinMode(PumpR_PWM, OUTPUT);
  pinMode(PumpL_PWM, OUTPUT);

  pinMode(SLeftUpButton, INPUT);
  pinMode(SLeftDownButton, INPUT);

  pinMode(SRightUpButton, INPUT);
  pinMode(SRightDownButton, INPUT);

  pinMode(PIN_RELAY_LED_1, OUTPUT);
  pinMode(PIN_RELAY_LED_2, OUTPUT);

  digitalWrite(MLeftR_PWM, HIGH);
  digitalWrite(MLeftL_PWM, HIGH);  // STOP

  digitalWrite(MRightL_PWM, HIGH);
  digitalWrite(MRightR_PWM, HIGH);  // STOP

  digitalWrite(SLeftR_PWM, HIGH);
  digitalWrite(SLeftL_PWM, HIGH);  // STOP

  digitalWrite(SRightL_PWM, HIGH);
  digitalWrite(SRightR_PWM, HIGH);  // STOP

  digitalWrite(CylR_PWM, HIGH);
  digitalWrite(CylL_PWM, HIGH);  // STOP

  digitalWrite(PumpR_PWM, HIGH);
  digitalWrite(PumpL_PWM, LOW);
  analogWrite(PumpRL_EN, 0);  // STOP

  digitalWrite(PIN_RELAY_LED_1, HIGH);
  digitalWrite(PIN_RELAY_LED_2, HIGH);
}

void loop() {
  if(needReset == true){
    delay(3000);
  }
  readAndSendDataToESP();
  if (Serial3.available()) {
    receiveDataAndRunCommands();
  }
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void readAndSendDataToESP() {
  // cam bien DHT
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);

  // cam bien anh sang
  int anaValue = analogRead(photodiodePin);
  float voltage = 5 - (anaValue / 1024.0) * 5;
  float lux = 11.52 * voltage;

  // cam bien MQ135
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(t, h);

  // cam bien TDS
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                   // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (t - 25.0);                                                                                                                          // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                             // temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;  // convert voltage value to tds value
  }

  // cam bien pH
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) {
    timepoint = millis();
    volt = analogRead(PH_PIN) / 1024.0 * 5000;
    phValue = water_ph.readPH(volt, t);
  }
  // water_ph.calibration(voltage, t);

  static unsigned long sendTimepoint = millis();
  if (millis() - sendTimepoint > 5000U) {
    sendTimepoint = millis();
    // Create a JSON object
    StaticJsonDocument<200> jsonDoc;

    // Populate the JSON object with sensor data
    jsonDoc["type"] = "sensors_data";
    jsonDoc["PH"] = phValue;
    jsonDoc["TDS"] = tdsValue;
    jsonDoc["Lux"] = lux;
    jsonDoc["Humidity"] = h;
    jsonDoc["Temperature"] = t;
    jsonDoc["Heat Index"] = hic;
    jsonDoc["Corrected_PPM"] = correctedPPM;
    jsonDoc["Dust Density"] = dustSensor.getDustDensity();
    jsonDoc["Running Average"] = dustSensor.getRunningAverage();

    // Serialize the JSON to a string
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Send the JSON string over Serial3 to the ESP8266
    Serial3.print("Sensors_data: " + jsonString + "\n");
    // Serial.print("Sensors_data: " + jsonString + "\n");
  }
}

void receiveDataAndRunCommands() {
  String data = Serial3.readStringUntil('\n');
  Serial.println(data);
  DynamicJsonDocument doc(256);  // Adjust buffer size as needed
  deserializeJson(doc, data);

  String eventType = doc["type"].as<String>();
  if (eventType.indexOf("control") != -1) {
    String commandType = eventType.substring(eventType.indexOf("/") + 1);
    int direction = doc["direction"].as<int>();

    if (commandType == "both_motors") {
      int speed1 = doc["speed1"].as<int>();
      int speed2 = doc["speed2"].as<int>();
      bothMotorsWithSpeed(direction, speed1, speed2);
    } else if (commandType == "left_motor") {
      leftMotor(direction);
    } else if (commandType == "right_motor") {
      rightMotor(direction);
    } else if (commandType == "cylinder") {
      cylinder(direction);
    } else if (commandType == "pump") {
      if (direction == 1) {
        bothServos(direction);  // both servos move to 90 deg for 3 secs, then pump starts
        delay(3000);
        pump(direction);
      } else {
        pump(direction);  // pump ends then both sevos move back down to 0 deg
        if (led_state == 0) {
          bothServos(direction);
        }
      }
    } else if (commandType == "both_leds") {
      if (direction == 1) {
        bothServos(direction);  // both servos move to 90 deg for 3 secs, then led starts
        delay(3000);
        bothLeds(direction);
      } else {
        bothLeds(direction);  // led ends then both sevos move back down to 0 deg
        if (pump_state == 0) {
          bothServos(direction);
        }
      }
    } else if (commandType == "led1") {
      led1(direction);
    } else if (commandType == "led2") {
      led2(direction);
    } else if (commandType == "left_servo") {
      leftServo(direction);
    } else if (commandType == "right_servo") {
      rightServo(direction);
    } else if (commandType == "both_servos") {
      bothServos(direction);
    } else if (commandType == "cylinder_to_read_soil") {
      cylinderToReadSoil();
    } else if (commandType == "read_soil") {
      readSoil(30000);
    }
  }
}

void leftMotor(int direction) {
  if (direction == 1) {
    digitalWrite(MLeftR_PWM, HIGH);  // clockwise
    digitalWrite(MLeftL_PWM, LOW);
    analogWrite(MLeftRL_EN, 80);
  } else if (direction == 0) {
    analogWrite(MLeftRL_EN, 0);
  } else {
    digitalWrite(MLeftR_PWM, LOW);  // counter-clockwise
    digitalWrite(MLeftL_PWM, HIGH);
    analogWrite(MLeftRL_EN, 80);
  }
}

void rightMotor(int direction) {
  if (direction == 1) {
    digitalWrite(MRightR_PWM, HIGH);  // clockwise
    digitalWrite(MRightL_PWM, LOW);
    analogWrite(MRightRL_EN, 80);
  } else if (direction == 0) {
    analogWrite(MRightRL_EN, 0);
  } else {
    digitalWrite(MRightL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MRightR_PWM, LOW);
    analogWrite(MRightRL_EN, 80);
  }
}

void bothMotors(int direction) {
  motors_state = direction;
  if (direction == 1) {
    digitalWrite(MLeftR_PWM, HIGH);  // clockwise
    digitalWrite(MLeftL_PWM, LOW);
    digitalWrite(MRightL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MRightR_PWM, LOW);
    analogWrite(MLeftRL_EN, 80);
    analogWrite(MRightRL_EN, 80);
  } else if (direction == 0) {
    analogWrite(MLeftRL_EN, 0);
    analogWrite(MRightRL_EN, 0);
  } else {
    digitalWrite(MLeftL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MLeftR_PWM, LOW);
    digitalWrite(MRightR_PWM, HIGH);  // clockwise
    digitalWrite(MRightL_PWM, LOW);
    analogWrite(MLeftRL_EN, 80);
    analogWrite(MRightRL_EN, 80);
  }
}

void bothMotorsWithSpeed(int direction, int speed1, int speed2) {
  motors_state = direction;
  left_motor_speed = speed1;
  right_motor_speed = speed2;
  if (direction == 1) {
    digitalWrite(MLeftR_PWM, HIGH);  // clockwise
    digitalWrite(MLeftL_PWM, LOW);
    digitalWrite(MRightL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MRightR_PWM, LOW);
  } else if (direction != 0) {
    digitalWrite(MLeftL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MLeftR_PWM, LOW);
    digitalWrite(MRightR_PWM, HIGH);  // clockwise
    digitalWrite(MRightL_PWM, LOW);
  }
  analogWrite(MLeftRL_EN, toSpeed(speed1));
  analogWrite(MRightRL_EN, toSpeed(speed2));
}

void cylinder(int direction) {
  cylinder_state = direction;
  Serial.println("cylinder");
  if (direction == 1) {
    digitalWrite(CylR_PWM, HIGH);  // extend
    digitalWrite(CylL_PWM, LOW);
    analogWrite(CylRL_EN, 255);
  } else if (direction == 0) {
    analogWrite(CylRL_EN, 0);
  } else {
    digitalWrite(CylR_PWM, LOW);  // retract
    digitalWrite(CylL_PWM, HIGH);
    analogWrite(CylRL_EN, 255);
  }
}

void pump(int direction) {
  pump_state = direction;
  if (direction == 1) {
    analogWrite(PumpRL_EN, 255);
  } else {
    analogWrite(PumpRL_EN, 0);
  }
}

void leftServo(int direction) {
  if (direction == 1)  // go up
  {
    digitalWrite(SLeftR_PWM, HIGH);  // clockwise
    digitalWrite(SLeftL_PWM, LOW);
    analogWrite(SLeftRL_EN, 210);
    while (digitalRead(SLeftUpButton) == 1) {
      delay(1);
    }
    analogWrite(SLeftRL_EN, 50);
  } else  // go down
  {
    digitalWrite(SLeftL_PWM, HIGH);  // counter-clockwise
    digitalWrite(SLeftR_PWM, LOW);
    analogWrite(SLeftRL_EN, 210);
    while (digitalRead(SLeftDownButton) == 1) {
      delay(1);
    }
    analogWrite(SLeftRL_EN, 0);
  }
}

void rightServo(int direction) {
  if (direction == 1) {
    digitalWrite(SRightL_PWM, HIGH);  // clockwise
    digitalWrite(SRightR_PWM, LOW);
    analogWrite(SRightRL_EN, 210);
    while (digitalRead(SRightUpButton) == 1) {
      delay(1);
    }
    analogWrite(SRightRL_EN, 50);
  } else {
    digitalWrite(SRightR_PWM, HIGH);  // counter-clockwise
    digitalWrite(SRightL_PWM, LOW);
    analogWrite(SRightRL_EN, 210);
    while (digitalRead(SRightDownButton) == 1) {
      delay(1);
    }
    analogWrite(SRightRL_EN, 0);
  }
}

void bothServos(int direction) {  //1: Up, 0:Down
  servos_state = direction;
  bothMotorsWithSpeed(0, left_motor_speed, right_motor_speed);
  if (direction == 1) {
    leftServo(1);
    rightServo(1);
  } else {
    leftServo(0);
    rightServo(0);
  }
  bothMotorsWithSpeed(motors_state, left_motor_speed, right_motor_speed);
}

void bothLeds(int direction) {  // On, Off
  led_state = direction;
  if (direction == 1) {
    digitalWrite(PIN_RELAY_LED_1, LOW);
    digitalWrite(PIN_RELAY_LED_2, LOW);
  } else {
    digitalWrite(PIN_RELAY_LED_1, HIGH);
    digitalWrite(PIN_RELAY_LED_2, HIGH);
  }
}

void led1(int direction) {
  led_state = direction;
  if (direction == 1) {
    digitalWrite(PIN_RELAY_LED_1, LOW);
  } else {
    digitalWrite(PIN_RELAY_LED_1, HIGH);
  }
}

void led2(int direction) {
  led_state = direction;
  if (direction == 1) {
    digitalWrite(PIN_RELAY_LED_2, LOW);
  } else {
    digitalWrite(PIN_RELAY_LED_2, HIGH);
  }
}

int toSpeed(int percent) {
  // Serial.println(round(abs((float)percent) / 100 * 255));
  return round(abs((float)percent) / 100 * 255);
}

void readSoil(int duration) {
  static int readSoilTimepoint = millis();
  while (millis() - readSoilTimepoint < duration) {
    Serial.println("Read Data NPK 7 in 1 SENSOR: ID = 1");
    node.begin(1, mySerial);
    delay(300);

    /*Read value PH*/
    result = node.readHoldingRegisters(0x0006, 2);
    // do something with data if read is successful
    if (result == node.ku8MBSuccess) {
      data[0] = node.receive();
      ph = float(data[0]);
      Serial.print("pH: ");
      Serial.print(ph / 100);
      Serial.println("pH");
    }
    delay(200);
    /*Read value do am*/
    result = node.readHoldingRegisters(0x0012, 2);
    // do something with data if read is successful
    if (result == node.ku8MBSuccess) {
      data[0] = node.receive();
      Humi = float(data[0]);
      Serial.print("Do am dat: ");
      Serial.print(Humi / 10);
      Serial.println("%");
    }
    delay(500);
    /*Read value nhiet do*/
    result = node.readHoldingRegisters(0x0013, 2);
    // do something with data if read is successful
    if (result == node.ku8MBSuccess) {
      data[0] = node.receive();
      Temp = float(data[0]);
      Serial.print("Nhiet do dat: ");
      Serial.print(Temp / 10);
      Serial.println("oC");
    }
    delay(200);

    /*Read value Nito*/
    result = node.readHoldingRegisters(0x001E, 2);
    // do something with data if read is successful
    if (result == node.ku8MBSuccess) {
      data[0] = node.receive();
      Nito = float(data[0]);
      Serial.print("Nito Value: ");
      Serial.print(Nito);
      Serial.println("mg/Kg");
    }
    delay(200);
    /*Read value Photpho*/
    result = node.readHoldingRegisters(0x001F, 2);
    // do something with data if read is successful
    if (result == node.ku8MBSuccess) {
      data[0] = node.receive();
      Photpho = float(data[0]);
      Serial.print("Photpho Value: ");
      Serial.print(Photpho);
      Serial.println("mg/Kg");
    }
    delay(200);

    /*Read value Kali*/
    result = node.readHoldingRegisters(0x0020, 2);
    // do something with data if read is successful
    if (result == node.ku8MBSuccess) {
      data[0] = node.receive();
      Kali = float(data[0]);
      Serial.print("Kali Value: ");
      Serial.print(Kali);
      Serial.println("mg/Kg");
    }
    /*Read value do dan dien*/
    result = node.readHoldingRegisters(0x0021, 2);
    // do something with data if read is successful
    if (result == node.ku8MBSuccess) {
      data[0] = node.receive();
      EC = float(data[0]);
      Serial.print("Do dan dien: ");
      Serial.print(EC);
      Serial.println("us/cm");
    }
    delay(200);

    StaticJsonDocument<200> jsonDoc;

    // Populate the JSON object with sensor data
    jsonDoc["type"] = "sensors_data";
    jsonDoc["pHsoil"] = ph / 100;
    jsonDoc["Humsoil"] = Humi / 10;
    jsonDoc["Tempsoil"] = Temp / 10;
    jsonDoc["Nitrogen"] = Nito;
    jsonDoc["Phosphorus"] = Photpho;
    jsonDoc["Potassium"] = Kali;
    jsonDoc["EC"] = EC;

    // Serialize the JSON to a string
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Send the JSON string over Serial3 to the ESP8266
    Serial3.print("Sensors_data: " + jsonString + "\n");
  }
}

void cylinderToReadSoil() {
  bothMotorsWithSpeed(0, left_motor_speed, right_motor_speed);
  delay(1000);
  cylinder(-1);
  delay(7000);
  readSoil(300000);
  cylinder(1);
  delay(5000);
  cylinder(0);
  delay(1000);
  bothMotorsWithSpeed(motors_state, left_motor_speed, right_motor_speed);
}
