#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "MQ135.h"
#include <GP2YDustSensor.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
// #include <Servo.h>
#define PIN_MQ135 A3
#define DHTPIN 10
#define photodiodePin A2
#define DHTTYPE DHT22
#define TdsSensorPin A1
#define VREF 5.0
#define SCOUNT 30
#define PH_PIN A0
#define SHARP_LED_PIN 31
#define SHARP_VO_PIN A4
#define MRightRL_EN 5   // băm xung
#define MRightR_PWM 28  // điều khiển chiều quay
#define MRightL_PWM 29  // điều khiển chiều quay
#define CylRL_EN 2      // băm xung
#define CylR_PWM 22     // điều khiển chiều quay
#define CylL_PWM 23   
  // điều khiển chiều quay
#define MLeftRL_EN 4    // băm xung
#define MLeftR_PWM 26   // điều khiển chiều quay
#define MLeftL_PWM 27   // điều khiển chiều quay
#define PumpRL_EN 3     // băm xung
#define PumpR_PWM 24    // điều khiển chiều quay
#define PumpL_PWM 25    // điều khiển chiều quay

#define SLeftRL_EN 6
#define SLeftR_PWM 38  // điều khiển chiều quay
#define SLeftL_PWM 39  // điều khiển chiều quay
#define SRightRL_EN 7
#define SRightR_PWM 40  // điều khiển chiều quay
#define SRightL_PWM 41  // điều khiển chiều quay
#define SLeftUpButton 34     // to see if left servo has fully turned 90 deg
#define SLeftDownButton 35   // to see if left servo has fully turned 0 deg
#define SRightUpButton 36    // to see if right servo has fully turned 90 deg
#define SRightDownButton 37  // to see if eight servo has fully turned 0 deg
#define PIN_RELAY_LED_1 30
#define PIN_RELAY_LED_2 31
#define PIN_RELAY_3 32
#define PIN_RELAY_4 33

float volt, phValue = 25;
DFRobot_PH ph;
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;

MQ135 mq135_sensor = MQ135(PIN_MQ135);
DHT dht = DHT(DHTPIN, DHTTYPE);
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1014AU0F, SHARP_LED_PIN, SHARP_VO_PIN);
// Servo servo1;
// Servo servo2;

static int left_servo_state, right_servo_state, servos_state, motors_state, pump_state, led_state, cylinder_state;  //= direction

// #define SSerialRX         2                    // RS485 Serial Receive pin
// #define SSerialTX         3                    // RS485 Serial Transmit pin
// SoftwareSerial mySerial (SSerialRX, SSerialTX);
// ModbusMaster node;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial3.begin(9600);

  Serial.println("\n[SETUP] BOOT");
  pinMode(TdsSensorPin, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(PIN_MQ135, INPUT);
  pinMode(photodiodePin, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(SHARP_LED_PIN, INPUT);
  pinMode(SHARP_VO_PIN, INPUT);
  dht.begin();
  ph.begin();
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
  digitalWrite(PumpL_PWM, HIGH);  // STOP

  pinMode(SLeftUpButton, INPUT);
  pinMode(SLeftDownButton, INPUT);
  pinMode(SRightUpButton, INPUT);
  pinMode(SRightDownButton, INPUT);

  pinMode(PIN_RELAY_LED_1, OUTPUT);
  pinMode(PIN_RELAY_LED_2, OUTPUT);
  pinMode(PIN_RELAY_3, OUTPUT);
  pinMode(PIN_RELAY_4, OUTPUT);
  digitalWrite(PIN_RELAY_LED_1, HIGH);
  digitalWrite(PIN_RELAY_LED_2, HIGH);
  cylinder(-1);
  delay(4000);
  cylinder(0);
}

void loop() {
  // bothMotors(1);
  // delay(5000);
  // bothMotors(-1);
  // delay(5000);
  // if (Serial3.available()) {
  //   receiveDataAndRunCommands();
  // }
  // readAndSendDataToESP();

}

void test_run() {
  delay(5000);
  bothServos(0);
  delay(5000);
  bothServos(1);
  delay(5000);
  bothMotors(2);
  delay(5000);
  bothMotors(0);
  delay(5000);
  bothMotors(1);
  delay(5000);
  cylinder(2);
  delay(5000);
  cylinder(0);
  delay(5000);
  cylinder(1);
  delay(5000);
  pump(1);
  delay(5000);
  pump(0);
  delay(5000);
  led1(1);
  delay(5000);
  led1(0);
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
    float compensationCoefficient = 1.0 + 0.02 * (t - 25.0);                                                                                                                          // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                             // temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;  // convert voltage value to tds value
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
  if (direction == 1) {
    digitalWrite(MLeftR_PWM, HIGH);  // clockwise
    digitalWrite(MLeftL_PWM, LOW);
  } else {
    digitalWrite(MLeftR_PWM, LOW);  // counter-clockwise
    digitalWrite(MLeftL_PWM, HIGH);
  }
  analogWrite(MLeftRL_EN, 80);
  delay(duration);
  analogWrite(MLeftRL_EN, 0);
}

void motorRightForDuration(int direction, int duration) {
  if (direction == 1) {
    digitalWrite(MRightR_PWM, HIGH);  // clockwise
    digitalWrite(MRightL_PWM, LOW);
  } else {
    digitalWrite(MRightL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MRightR_PWM, LOW);
  }
  analogWrite(MRightRL_EN, 80);
  delay(duration);
  analogWrite(MRightRL_EN, 0);
}

void bothMotorsForDuration(int direction, int duration) {
  if (direction == 1) {
    digitalWrite(MLeftR_PWM, HIGH);  // clockwise
    digitalWrite(MLeftL_PWM, LOW);
    digitalWrite(MRightR_PWM, LOW);
    digitalWrite(MRightL_PWM, HIGH);  // counter-clockwise
  } else {
    digitalWrite(MRightR_PWM, HIGH);  // clockwise
    digitalWrite(MRightL_PWM, LOW);
    digitalWrite(MLeftL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MLeftR_PWM, LOW);
  }
  analogWrite(MLeftRL_EN, 80);
  analogWrite(MRightRL_EN, 80);
  delay(duration);
  analogWrite(MLeftRL_EN, 0);
  analogWrite(MRightRL_EN, 0);
}

void cylinderForDuration(int direction, int duration) {
  if (direction == 1) {
    digitalWrite(CylR_PWM, HIGH);  // up
    digitalWrite(CylL_PWM, LOW);
  } else {
    digitalWrite(CylR_PWM, LOW);  // down
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
    Serial.println("run forward");
    digitalWrite(MLeftR_PWM, HIGH);  // clockwise
    digitalWrite(MLeftL_PWM, LOW);
    digitalWrite(MRightL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MRightR_PWM, LOW);
    analogWrite(MLeftRL_EN, 80);
    analogWrite(MRightRL_EN, 80);
  } else if (direction == 0) {
    Serial.println("stopped");
    analogWrite(MLeftRL_EN, 0);
    analogWrite(MRightRL_EN, 0);
  } else {
    Serial.println("run backward");
    digitalWrite(MLeftL_PWM, HIGH);  // counter-clockwise
    digitalWrite(MLeftR_PWM, LOW);
    digitalWrite(MRightR_PWM, HIGH);  // clockwise
    digitalWrite(MRightL_PWM, LOW);
    analogWrite(MLeftRL_EN, 80);
    analogWrite(MRightRL_EN, 80);
  }
}

void cylinder(int direction) {
  cylinder_state = direction;
  if (direction == 1) {
    digitalWrite(CylR_PWM, HIGH);  // up
    digitalWrite(CylL_PWM, LOW);
    analogWrite(CylRL_EN, 255);
  } else if (direction == 0) {
    analogWrite(CylRL_EN, 0);
  } else {
    digitalWrite(CylR_PWM, LOW);  // down
    digitalWrite(CylL_PWM, HIGH);
    analogWrite(CylRL_EN, 255);
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

// void bothServos(int direction)
// { // On: 90, Off: 0
//   servos_state = direction;
//   if(motors_state != 0){
//     bothMotors(0);
//   }
//   if (direction == 1)
//   {
//     servo1.write(95);
//     servo2.write(95);
//   }
//   else
//   {
//     servo1.write(0);
//     servo2.write(0);
//   }
//   bothMotors(motors_state);
// }

void controlLeftServo(int speed) {
  if (speed >= 0) {
    digitalWrite(SLeftR_PWM, HIGH);  // clockwise
    digitalWrite(SLeftL_PWM, LOW);
  } else {
    digitalWrite(SLeftL_PWM, HIGH);  // counter-clockwise
    digitalWrite(SLeftR_PWM, LOW);
    speed = -speed;  // Make speed positive for analogWrite
  }

  analogWrite(SLeftRL_EN, speed);
}

void controlRightServo(int speed) {
  if (speed >= 0) {
    digitalWrite(SRightL_PWM, HIGH);  // clockwise
    digitalWrite(SRightR_PWM, LOW);
  } else {
    digitalWrite(SRightR_PWM, HIGH);  // counter-clockwise
    digitalWrite(SRightL_PWM, LOW);
    speed = -speed;  // Make speed positive for analogWrite
  }

  analogWrite(SRightRL_EN, speed);
}

void leftServo(int direction) {
  if (direction == 1)  // go up
  {
    digitalWrite(SLeftR_PWM, HIGH);  // clockwise
    digitalWrite(SLeftL_PWM, LOW);
    analogWrite(SLeftRL_EN, 130);
    while (digitalRead(SLeftUpButton) == 1) {
      delay(1);
    }
    analogWrite(SLeftRL_EN, 0);
  } else  // go down
  {
    digitalWrite(SLeftL_PWM, HIGH);  // counter-clockwise
    digitalWrite(SLeftR_PWM, LOW);
    analogWrite(SLeftRL_EN, 130);
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
    analogWrite(SRightRL_EN, 130);
    while (digitalRead(SRightUpButton) == 1) {
      delay(1);
    }
    analogWrite(SRightRL_EN, 0);
  } else {
    digitalWrite(SRightR_PWM, HIGH);  // counter-clockwise
    digitalWrite(SRightL_PWM, LOW);
    analogWrite(SRightRL_EN, 130);
    while (digitalRead(SRightDownButton) == 1) {
      delay(1);
    }
    analogWrite(SRightRL_EN, 0);
  }
}

void bothServos(int direction) {
  servos_state = direction;
  bothMotors(0);
  if (direction == 1) {
    leftServo(1);
    rightServo(1);
  } else {
    leftServo(0);
    rightServo(0);
  }
  bothMotors(motors_state);
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
      Serial.println("running motors");
    } else if (commandType == "cylinder") {
      bothMotors(0);
      cylinder(direction);
    } else if (commandType == "pump") {
      if (direction == 1) {
        bothServos(direction);  // both servos move to 90 deg for 10 secs, then pump starts
        delay(3000);
        pump(direction);
      } else {
        pump(direction);  // pump ends then both sevos move back down to 0 deg
        if (led_state == 0) {
          bothServos(direction);
        }
      }
    } else if (commandType == "led") {
      if (direction == 1) {
        bothServos(direction);  // both servos move to 90 deg for 10 secs, then led starts
        delay(3000);
        led1(direction);
      } else {
        led1(direction);  // led ends then both sevos move back down to 0 deg
        if (pump_state == 0) {
          bothServos(direction);
        }
      }
    }
  }
}
