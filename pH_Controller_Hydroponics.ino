#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>
#include <SD.h>
#include "DFRobot_PH.h"
#include "WiFiS3.h"
#include "arduino_secrets.h" 
#include "GravityRtc.h"

// Netzwerkdaten
char ssid[] = SECRET_SSID;        // Ihr Netzwerkname (SSID)
char pass[] = SECRET_PASS;        // Ihr Netzwerkpasswort

WiFiServer server(80);

// Konstanten und Variablen
#define TEMPERATURE_SENSOR_PIN 2
#define TDS_SENSOR_PIN A1
#define PH_SENSOR_PIN A2
#define SET_POINT_PH 5.8
#define PH_THRESHOLD 0.15
#define UPDATE_INTERVAL 5000
#define PH_MEASUREMENT_INTERVAL 120000
#define SD_CARD_CS_PIN 5
#define DATA_LOG_INTERVAL 900000
#define SCOUNT 30 // Anzahl der Messpunkte für den Medianfilter
#define BUTTON_PIN_1 3  // Taster 1 an Pin D3
#define BUTTON_PIN_2 4  // Taster 2 an Pin D4


#define PH_BUFFER_SIZE 10 // Größe des Buffers für den gleitenden Mittelwert
float phValues[PH_BUFFER_SIZE]; // Array zum Speichern der letzten pH-Werte
int phValueIndex = 0; // Index für das pH-Werte-Array
float averagePH = 0.0; // Variable für den gleitenden Mittelwert des pH-Wertes

DFRobot_PH ph;
OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Servo pump1;
Servo pump2;
boolean isPump1Attached = false;
boolean isPump2Attached = false;

float firstPHValue;
boolean isFirstPHValueMeasured = false;
unsigned long lastPHMeasurementTime = 0;
unsigned long lastDataLogTime = 0;
File dataFile;
float globalEcValue = 0.0; // Globale Variable für den EC-Wert
float currentTemperature = 0.0; // Globale Variable für die Temperatur
float currentPHValue = 0.0; // Globale Variable für den pH-Wert

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
unsigned long previousTdsSampleTime = 0;

GravityRtc rtc;     // RTC-Objekt initialisieren

void setup() {
  Serial.begin(9600);

  // RTC initialisieren
  rtc.setup();

  // Versuchen, eine WiFi-Verbindung herzustellen
  Serial.println("Verbinde mit WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(5000);
    Serial.print(".");
  }
  Serial.println("Verbunden mit WiFi");
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Adresse: ");
  Serial.println(ip);

  // Starten des Webservers
  server.begin();
  Serial.println("Webserver gestartet");

  sensors.begin();
  u8g2.begin(); // OLED Display initialisieren
  ph.begin();

  if (!SD.begin(SD_CARD_CS_PIN)) {
    Serial.println("SD-Karteninitialisierung fehlgeschlagen!");
    return;
  }
  Serial.println("SD-Karte ist bereit.");
}

void loop() {
  unsigned long currentTime = millis();

  // Aktualisieren Sie die Sensordaten
  updateSensorValues();

  // Anzeigen der Sensorwerte auf dem OLED
  displaySensorValues();

  controlPumpsWithButtons(); 



  if (!isFirstPHValueMeasured) {
    firstPHValue = averagePH; // Verwenden Sie averagePH anstelle von currentPHValue
    isFirstPHValueMeasured = true;
    lastPHMeasurementTime = currentTime;
  } else if (currentTime - lastPHMeasurementTime >= PH_MEASUREMENT_INTERVAL) {
    float pHDifference = abs(averagePH - firstPHValue);
    if (pHDifference <= PH_THRESHOLD) {
      controlPumps(averagePH);
    }
    isFirstPHValueMeasured = false;
  }

  if (currentTime - lastDataLogTime >= DATA_LOG_INTERVAL) {
    rtc.read();
    String timeStamp = String(rtc.year) + "-" + String(rtc.month) + "-" + String(rtc.day) + " " + String(rtc.hour) + ":" + String(rtc.minute) + ":" + String(rtc.second);
    saveDataToSDCard(timeStamp);
    lastDataLogTime = currentTime;
  }

  handleClient(averagePH);
  delay(UPDATE_INTERVAL);
}

void updateSensorValues() {
  // Temperaturmessung
  sensors.requestTemperatures();
  currentTemperature = sensors.getTempCByIndex(0);

  // EC-Wert Messung
  if (millis() - previousTdsSampleTime > 40) {
    previousTdsSampleTime = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_SENSOR_PIN);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  float averageVoltage = getMedianNum(analogBuffer, SCOUNT) * (5.0 / 1024.0);
  float compensationCoefficient = 1.0 + 0.02 * (currentTemperature - 25.0);
  float compensationVoltage = averageVoltage / compensationCoefficient;
  float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
  globalEcValue = tdsValue * 2.0 / 1000.0; // EC-Wert

  // pH-Wert Messung
  float voltage = analogRead(PH_SENSOR_PIN) / 1024.0 * 5000;
  currentPHValue = ph.readPH(voltage, currentTemperature);

  // Gleitenden Mittelwert des pH-Wertes berechnen
  phValues[phValueIndex] = currentPHValue;
  phValueIndex = (phValueIndex + 1) % PH_BUFFER_SIZE;

  float sum = 0.0;
  for (int i = 0; i < PH_BUFFER_SIZE; i++) {
    sum += phValues[i];
  }
  averagePH = sum / PH_BUFFER_SIZE;
}

void displaySensorValues() {
  u8g2.clearBuffer(); 
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  u8g2.setCursor(0, 10);
  u8g2.print("Temp: ");
  u8g2.print(currentTemperature);
  u8g2.print(" C");

  u8g2.setCursor(0, 20);
  u8g2.print("EC: ");
  u8g2.print(globalEcValue, 2);

  u8g2.setCursor(0, 30);
  u8g2.print("pH: ");
  u8g2.print(averagePH);

  u8g2.sendBuffer();
}

void handleClient(float pHValue) {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  String req = "";
  while (client.available()) {
    char c = client.read();
    if (c == '\r' || c == '\n') break;
    req += c;
  }
  client.flush();

  if (req.indexOf("/dataLog.txt") != -1) {
    sendDataLog(client);
  } else {
    sendWebPage(client, pHValue);
  }
  client.stop();
}

void sendWebPage(WiFiClient& client, float pHValue) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<head><title>pH-Controller Dashboard</title></head>");
  client.println("<body>");
  client.println("<h1>Arduino Sensor Dashboard</h1>");
  client.println("<div class='sensor-value'>Temperatur: <span id='temperature'>" + String(sensors.getTempCByIndex(0)) + "</span> °C</div>");
  client.println("<div class='sensor-value'>EC: <span id='ec'>" + String(globalEcValue) + "</span></div>");
  client.println("<div class='sensor-value'>pH: <span id='ph'>" + String(pHValue) + "</span></div>");
  client.println("<a href=\"/dataLog.txt\" download=\"dataLog.txt\"><button>Download Protokoll</button></a>");
  client.println("</body>");
  client.println("</html>");
}

void sendDataLog(WiFiClient& client) {
  dataFile = SD.open("dataLog.txt");
  if (dataFile) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Content-Disposition: attachment; filename=\"dataLog.txt\"");
    client.println("Connection: close");
    client.println();
    while (dataFile.available()) {
      client.write(dataFile.read());
    }
    dataFile.close();
  } else {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Connection: close");
    client.println();
    client.println("Datei nicht gefunden.");
  }
}

void measureAndDisplay() {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  if (millis() - previousTdsSampleTime > 40) {
    previousTdsSampleTime = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_SENSOR_PIN);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  float averageVoltage = getMedianNum(analogBuffer, SCOUNT) * (5.0 / 1024.0);
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = averageVoltage / compensationCoefficient;
  float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
  float ecValue = tdsValue * 2.0 / 1000.0;

  globalEcValue = ecValue; // Aktualisieren der globalen EC-Wert-Variable

  // Aktualisierte Code für OLED-Display
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  u8g2.setCursor(0, 10);
  u8g2.print("Temp: ");
  u8g2.print(temperature);
  u8g2.print(" C");

  u8g2.setCursor(0, 20);
  u8g2.print("EC: ");
  u8g2.print(ecValue, 2);

  u8g2.sendBuffer();
}


float measurePH() {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  float voltage = analogRead(PH_SENSOR_PIN)/1024.0*5000;
  return ph.readPH(voltage, temperature);
}

void controlPumps(float pHValue) {
  if (pHValue > SET_POINT_PH + PH_THRESHOLD && !isPump1Attached) {
    pump1.attach(8); // Pump right PH-Minus
    isPump1Attached = true;
    pump1.write(170);
    delay(100);
    pump1.write(90);
    pump1.detach();
    isPump1Attached = false;
  } else if (pHValue < SET_POINT_PH - PH_THRESHOLD && !isPump2Attached) {
    pump2.attach(9); // Pump left PH-Plus
    isPump2Attached = true;
    pump2.write(10);
    delay(100);
    pump2.write(90);
    pump2.detach();
    isPump2Attached = false;
  }
}

void controlPumpsWithButtons() {
  int buttonState1 = digitalRead(BUTTON_PIN_1);
  int buttonState2 = digitalRead(BUTTON_PIN_2);

  Serial.print("Button 1 State: ");
  Serial.println(buttonState1);
  Serial.print("Button 2 State: ");
  Serial.println(buttonState2);

  // Überprüfen Sie, ob der Button gedrückt ist (LOW bei INPUT_PULLUP)
  if (buttonState1 == LOW) {
    pump1.attach(8); // Pumpe rechts PH-Plus
    isPump1Attached = true;
    pump1.write(160); // Pumpe an
    Serial.println("Pump 1 ON");
  } else {
    pump1.write(90); // Pumpe aus
    Serial.println("Pump 1 OFF");
    pump1.detach();
    isPump1Attached = false;
  }

  if (buttonState2 == LOW) {
    pump2.attach(9); // Pumpe links PH-Minus
    isPump2Attached = true;
    pump2.write(20); // Pumpe an
    Serial.println("Pump 2 ON");
  } else {
    pump2.write(90); // Pumpe aus
    Serial.println("Pump 2 OFF");
    pump2.detach();
    isPump2Attached = false;
  }
}



void saveDataToSDCard(String timeStamp) {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  float pHValue = measurePH();

  dataFile = SD.open("dataLog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Zeitstempel: ");
    dataFile.println(timeStamp);  // Zeitstempel hinzufügen
    dataFile.print("Temperatur: ");
    dataFile.print(temperature);
    dataFile.print(", EC: ");
    dataFile.print(globalEcValue, 2);
    dataFile.print(", pH: ");
    dataFile.println(pHValue);
    dataFile.close();
    Serial.println("Daten gespeichert.");
  } else {
    Serial.println("Fehler beim Öffnen der dataLog.txt");
  }
}

float getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (int i = 0; i < iFilterLen; i++) {
    bTab[i] = bArray[i];
  }
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
