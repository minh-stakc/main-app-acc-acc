#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "MQ135.h"
#include <GP2YDustSensor.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <Servo.h>
#define PIN_MQ135 A3
#define DHTPIN 8
#define photodiodePin A2
#define DHTTYPE DHT22
#define TdsSensorPin A1
#define VREF 5.0
#define SCOUNT 30
#define PH_PIN A0
#define Servo1Pin 6
#define Servo2Pin 7

#define MLeftRL_EN 3    // băm xung
#define MLeftR_PWM 24   // điều khiển chiều quay
#define MLeftL_PWM 25   // điều khiển chiều quay
#define MRightRL_EN 5   // băm xung
#define MRightR_PWM 28  // điều khiển chiều quay
#define MRightL_PWM 29  // điều khiển chiều quay
#define CylRL_EN 2      // băm xung
#define CylR_PWM 22     // điều khiển chiều quay
#define CylL_PWM 23     // điều khiển chiều quay
#define PumpRL_EN 4     // băm xung
#define PumpR_PWM 26    // điều khiển chiều quay
#define PumpL_PWM 27    // điều khiển chiều quay

#define PIN_RELAY_LED_1 30
#define PIN_RELAY_LED_2 31
#define PIN_RELAY_3 32
#define PIN_RELAY_4 33
#define SHARP_LED_PIN 31
#define SHARP_VO_PIN A4

// int bts_ports[4][3] = {{MLeftRL_EN,MLeftR_PWM, MLeftL_PWM}, {MRightRL_EN, MRightR_PWM, MRightL_PWM}, {CylRL_EN,CylR_PWM, CylL_PWM}, {PumpRL_EN, PumpR_PWM, PumpL_PWM}};

float volt, phValue = 25;
DFRobot_PH ph;
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;
MQ135 mq135_sensor = MQ135(PIN_MQ135);
DHT dht = DHT(DHTPIN, DHTTYPE);
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1014AU0F, SHARP_LED_PIN, SHARP_VO_PIN);

static unsigned int servos_state = false;    //true: up, false: down;
static unsigned int motors_state = false;    //true: running, false: stopped
static unsigned int pump_state = false;      //true:on, false: off
static unsigned int led_state = false;       //true:on, false: off
static unsigned int cylinder_state = false;  //true:on, false: off


Servo servo1;
Servo servo2;
// #define SSerialRX         2                    // RS485 Serial Receive pin
// #define SSerialTX         3                    // RS485 Serial Transmit pin
// SoftwareSerial mySerial (SSerialRX, SSerialTX);
// ModbusMaster node;

void setup() {
  Serial.begin(9600);
  Serial.println("\n[SETUP] BOOT");
  Serial1.begin(9600);
  Serial3.begin(9600);
  // Serial2.begin(9600);
  dht.begin();
  dustSensor.begin();
  pinMode(TdsSensorPin, INPUT);
  ph.begin();
  pinMode(MLeftRL_EN, OUTPUT);
  pinMode(MLeftR_PWM, OUTPUT);
  pinMode(MLeftL_PWM, OUTPUT);
  pinMode(MRightRL_EN, OUTPUT);
  pinMode(MRightL_PWM, OUTPUT);
  pinMode(MRightR_PWM, OUTPUT);
  pinMode(CylRL_EN, OUTPUT);
  pinMode(CylR_PWM, OUTPUT);
  pinMode(CylL_PWM, OUTPUT);
  pinMode(PumpRL_EN, OUTPUT);
  pinMode(PumpR_PWM, OUTPUT);
  pinMode(PumpL_PWM, OUTPUT);
  digitalWrite(MLeftR_PWM, HIGH);
  digitalWrite(MLeftL_PWM, HIGH);  //STOP
  digitalWrite(MRightL_PWM, HIGH);
  digitalWrite(MRightR_PWM, HIGH);  //STOP
  digitalWrite(CylR_PWM, HIGH);
  digitalWrite(CylL_PWM, HIGH);  //STOP
  digitalWrite(PumpR_PWM, HIGH);
  digitalWrite(PumpL_PWM, HIGH);  //STOP
  servo1.attach(Servo1Pin);
  servo2.attach(Servo2Pin);
  pinMode(PIN_RELAY_LED_1, OUTPUT);
  pinMode(PIN_RELAY_LED_2, OUTPUT);
  pinMode(PIN_RELAY_3, OUTPUT);
  pinMode(PIN_RELAY_4, OUTPUT);
  digitalWrite(PIN_RELAY_LED_1, HIGH);
  digitalWrite(PIN_RELAY_LED_2, HIGH);
}

void loop() {
  // bothServos(0);
  if (Serial3.available()) {
    receiveDataAndRunCommands();
  }
  readAndSendDataToESP();
  // delay(5000);
  // bothMotors(2);
  // delay(5000);
  // bothMotors(0);
  // delay(5000);
  // bothMotors(1);
  // delay(5000);
  // cylinder(2);
  // delay(5000);
  // cylinder(0);
  // delay(5000);
  // cylinder(1);
  // delay(5000);
  // pump(1);
  // delay(5000);
  // pump(0);
  // delay(5000);
  // led1(1);
  // delay(5000);
  // led1(0);
}


void sendValues(float pH, float tds, float lux, float hum, float temp, float hic, float correctedPPM, float dust_density, float running_average) {
  // Serial.print("Sensors_data: ");
  // Serial.println(String(pH) + " " + String(tds) + " " + String(lux) + " " + String(hum) + " " + String(temp) + " " + String(hic) + " " + String(correctedPPM) + " " + String(dust_density) + " " + String(running_average));

  Serial1.print("Sensors_data: ");
  Serial1.println(String(pH) + " " + String(tds) + " " + String(lux) + " " + String(hum) + " " + String(temp) + " " + String(hic) + " " + String(correctedPPM) + " " + String(dust_density) + " " + String(running_average));
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
  float hif = dht.computeHeatIndex(f, h);
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
    float compensationCoefficient = 1.0 + 0.02 * (t - 25.0);                                                                                                                          //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                             //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;  //convert voltage value to tds value
  }


  // cam bien pH
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) {
    timepoint = millis();
    volt = analogRead(PH_PIN) / 1024.0 * 5000;
    phValue = ph.readPH(volt, t);
  }
  // ph.calibration(voltage, t);

  static unsigned long sendTimepoint = millis();
  if (millis() - sendTimepoint > 5000U) {
    sendTimepoint = millis();
    sendValues(phValue, tdsValue, lux, h, t, hic, correctedPPM, dustSensor.getDustDensity(), dustSensor.getRunningAverage());
  }
  // uint16_t data[6];
  // float Temp, Humi;           //Sensor Temperarure and Humidity
  // float Nito, Photpho, Kali;  // Smart water sensor PH/ORP
  // float result;


  // Serial.println("Read Data NPK SENSOR: ID = 1");
  // node.begin(1, mySerial);
  // delay(300);

  // /*Read value Nito*/
  // result = node.readHoldingRegisters(0x001E, 2);
  // // do something with data if read is successful
  // if (result == node.ku8MBSuccess) {
  //   data[0] = node.receive();
  //   Nito = float(data[0]);
  // }
  // delay(500);
  // /*Read value Photpho*/
  // result = node.readHoldingRegisters(0x001F, 2);
  // // do something with data if read is successful
  // if (result == node.ku8MBSuccess) {
  //   data[0] = node.receive();
  //   Photpho = float(data[0]);
  // }
  // delay(200);

  // /*Read value Kali*/
  // result = node.readHoldingRegisters(0x0020, 2);
  // // do something with data if read is successful
  // if (result == node.ku8MBSuccess) {
  //   data[0] = node.receive();
  //   Kali = float(data[0]);
  // }
  // delay(4000);



  // /*Read value độ dẫn điện*/
  // result = node.readHoldingRegisters(0x0015, 2);
  // // do something with data if read is successful
  // if (result == node.ku8MBSuccess) {
  //   data[0] = node.receive();
  //   EC = float(data[0]);
  // }
  // delay(200);

  // /*Read value nhiệt độ đất*/
  // result = node.readHoldingRegisters(0x0013, 2);
  // // do something with data if read is successful
  // if (result == node.ku8MBSuccess) {
  //   data[0] = node.receive();
  //   temp = float(data[0]);
  // }
  // delay(200);



  // /*Read value độ ẩm đất*/
  // result = node.readHoldingRegisters(0x0012, 2);
  // // do something with data if read is successful
  // if (result == node.ku8MBSuccess) {
  //   data[0] = node.receive();
  //   hum = float(data[0]);
  // }
  // delay(200);


  // /*Read value pH đất*/
  // result = node.readHoldingRegisters(0x0006, 2);
  // // do something with data if read is successful
  // if (result == node.ku8MBSuccess) {
  //   data[0] = node.receive();
  //   pH = float(data[0]);
  // }
  // delay(200);
}


void motorLeftForDuration(int direction, int duration) {
  if (direction == true) {
    digitalWrite(MLeftR_PWM, HIGH);  //clockwise
    digitalWrite(MLeftL_PWM, LOW);
  } else {
    digitalWrite(MLeftR_PWM, LOW);  //counter-clockwise
    digitalWrite(MLeftL_PWM, HIGH);
  }
  analogWrite(MLeftRL_EN, 80);
  delay(duration);
  analogWrite(MLeftRL_EN, 0);
}

void motorRightForDuration(int direction, int duration) {
  if (direction == true) {
    digitalWrite(MRightR_PWM, HIGH);  //clockwise
    digitalWrite(MRightL_PWM, LOW);
  } else {
    digitalWrite(MRightL_PWM, HIGH);  //counter-clockwise
    digitalWrite(MRightR_PWM, LOW);
  }
  analogWrite(MRightRL_EN, 80);
  delay(duration);
  analogWrite(MRightRL_EN, 0);
}


void bothMotorsForDuration(int direction, int duration) {
  if (direction == true) {
    digitalWrite(MLeftR_PWM, HIGH);  //clockwise
    digitalWrite(MLeftL_PWM, LOW);
    digitalWrite(MRightR_PWM, LOW);
    digitalWrite(MRightL_PWM, HIGH);  //counter-clockwise
  } else {
    digitalWrite(MRightR_PWM, HIGH);  //clockwise
    digitalWrite(MRightL_PWM, LOW);
    digitalWrite(MLeftL_PWM, HIGH);  //counter-clockwise
    digitalWrite(MLeftR_PWM, LOW);
  }
  analogWrite(MLeftRL_EN, 80);
  analogWrite(MRightRL_EN, 80);
  delay(duration);
  analogWrite(MLeftRL_EN, 0);
  analogWrite(MRightRL_EN, 0);
}

void cylinderForDuration(int direction, int duration) {
  if (direction == true) {
    digitalWrite(CylR_PWM, HIGH);  //up
    digitalWrite(CylL_PWM, LOW);
  } else {
    digitalWrite(CylR_PWM, LOW);  //down
    digitalWrite(CylL_PWM, HIGH);
  }
  analogWrite(CylRL_EN, 255);
  delay(duration);
  analogWrite(CylRL_EN, 0);
}

void pumpForDuration(int duration) {
  digitalWrite(PumpR_PWM, HIGH);
  digitalWrite(PumpL_PWM, LOW);
  analogWrite(PumpRL_EN, 255);
  delay(duration);
  analogWrite(PumpRL_EN, 0);
}

void motorLeft(int direction) {
  if (direction == 2) {
    digitalWrite(MLeftR_PWM, HIGH);  //clockwise
    digitalWrite(MLeftL_PWM, LOW);
    analogWrite(MLeftRL_EN, 80);
  } else if (direction == 0) {
    digitalWrite(MLeftR_PWM, LOW);  //counter-clockwise
    digitalWrite(MLeftL_PWM, HIGH);
    analogWrite(MLeftRL_EN, 80);
  } else {
    analogWrite(MLeftRL_EN, 0);
  }
}

void motorRight(int direction) {
  if (direction == 2) {
    digitalWrite(MRightR_PWM, HIGH);  //clockwise
    digitalWrite(MRightL_PWM, LOW);
    analogWrite(MRightRL_EN, 80);
  } else if (direction == 0) {
    digitalWrite(MRightL_PWM, HIGH);  //counter-clockwise
    digitalWrite(MRightR_PWM, LOW);
    analogWrite(MRightRL_EN, 80);
  } else {
    analogWrite(MRightRL_EN, 0);
  }
}


void bothMotors(int direction) {
  motors_state = direction;
  if (direction == 2) {
    digitalWrite(MLeftR_PWM, HIGH);  //clockwise
    digitalWrite(MLeftL_PWM, LOW);
    digitalWrite(MRightL_PWM, HIGH);  //counter-clockwise
    digitalWrite(MRightR_PWM, LOW);
    analogWrite(MLeftRL_EN, 80);
    analogWrite(MRightRL_EN, 80);
  } else if (direction == 0) {
    digitalWrite(MRightR_PWM, HIGH);  //clockwise
    digitalWrite(MRightL_PWM, LOW);
    digitalWrite(MLeftL_PWM, HIGH);  //counter-clockwise
    digitalWrite(MLeftR_PWM, LOW);
    analogWrite(MLeftRL_EN, 80);
    analogWrite(MRightRL_EN, 80);
  } else {
    analogWrite(MLeftRL_EN, 0);
    analogWrite(MRightRL_EN, 0);
  }
}

void cylinder(int direction) {
  cylinder_state = direction;
  if (direction == 2) {
    digitalWrite(CylR_PWM, HIGH);  //up
    digitalWrite(CylL_PWM, LOW);
    analogWrite(CylRL_EN, 255);
  } else if (direction == 0) {
    digitalWrite(CylR_PWM, LOW);  //down
    digitalWrite(CylL_PWM, HIGH);
    analogWrite(CylRL_EN, 255);
  } else {
    analogWrite(CylRL_EN, 0);
  }
}

void pump(int direction) {
  pump_state = direction;
  if (direction == 1) {
    digitalWrite(PumpR_PWM, HIGH);
    digitalWrite(PumpL_PWM, LOW);
    analogWrite(PumpRL_EN, 255);
  } else {
    analogWrite(PumpRL_EN, 0);
  }
}

void bothServos(int direction) {  //On: 90, Off: 0
  servos_state = direction;
  if (direction == 1) {
    servo1.write(95);
    servo2.write(95);
  } else {
    servo1.write(0);
    servo2.write(0);
  }
}

void bothLeds(int direction) {  //On, Off
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

void receiveDataAndRunCommands() {
  String data = Serial3.readStringUntil('\n');
  Serial.println(data);
  DynamicJsonDocument doc(100);  // Adjust buffer size as needed
  deserializeJson(doc, data);
  // Serial.println(doc);
  String eventType = doc["type"].as<String>();
  if (eventType.indexOf("control") != -1) {
    String commandType = eventType.substring(eventType.indexOf("/") + 1);
    int direction = doc["direction"].as<int>();

    if (commandType == "both_motors") {
      bothMotors(direction);
    } else if (commandType == "cylinder") {
      bothMotors(1);
      cylinder(direction);
    } else if (commandType == "pump") {
      if (direction == 1) {
        bothServos(direction);  //both servos move to 90 deg for 10 secs, then pump starts
        delay(5000);
        pump(direction);
      } else {
        pump(direction);  //pump ends then both sevos move back down to 0 deg
        if (led_state == 0) {
          bothServos(direction);
        }
      }
    } else if (commandType == "led") {
      if (direction == 1) {
        bothServos(direction);  //both servos move to 90 deg for 10 secs, then led starts
        delay(3000);
        led1(direction);
      } else {
        led1(direction);  //led ends then both sevos move back down to 0 deg
        if (pump_state == 0) {
          bothServos(direction);
        }
      }
    }
  }
}
