#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>
#include <SD.h>
#include "DFRobot_PH.h"
#include "WiFiS3.h"
#include "arduino_secrets.h" 
#include "GravityRtc.h"

// Netzwerkdaten
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiServer server(80);

#define TEMPERATURE_SENSOR_PIN 2
#define TDS_SENSOR_PIN A1
#define PH_SENSOR_PIN A2
#define SET_POINT_PH 5.8
#define PH_THRESHOLD 0.10
#define UPDATE_INTERVAL 5000
#define PH_MEASUREMENT_INTERVAL 120000
#define SD_CARD_CS_PIN 5
#define DATA_LOG_INTERVAL 900000
#define SCOUNT 30 // Anzahl der Messpunkte für den Medianfilter
#define BUTTON_PIN_1 3  // Taster 1 an Pin D3 Pumpe starten
#define BUTTON_PIN_2 4  // Taster 2 an Pin D4 Pumpe starten
#define BUTTON_PIN_3 6  // Neuer Taster 3 an Pin D6 für pH erhöhen
#define BUTTON_PIN_4 7  // Neuer Taster 4 an Pin D7 für pH verringern
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)


Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

#define PH_BUFFER_SIZE 10
float phValues[PH_BUFFER_SIZE];
int phValueIndex = 0;
float averagePH = 0.0;
float setPointPH = SET_POINT_PH;

DFRobot_PH ph;
OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
Servo pump1;
Servo pump2;
boolean isPump1Attached = false;
boolean isPump2Attached = false;

float firstPHValue;
boolean isFirstPHValueMeasured = false;
unsigned long lastPHMeasurementTime = 0;
unsigned long lastDataLogTime = 0;
File dataFile;
float globalEcValue = 0.0;
float currentTemperature = 0.0;
float currentPHValue = 0.0;

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
unsigned long previousTdsSampleTime = 0;

GravityRtc rtc;

unsigned long correctionTimes[120]; // Array zum Speichern der Zeitstempel der letzten Korrekturen für jede Pumpe
int correctionIndex = 0; // Index für das correctionTimes Array
int minusCorrectionCount = 0; // Zähler für Korrekturen in der letzten Stunde für pH-Minus
int plusCorrectionCount = 0; // Zähler für Korrekturen in der letzten Stunde für pH-Plus


void setup() {
  Serial.begin(9600);
  rtc.setup();

  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(BUTTON_PIN_3, INPUT_PULLUP);
  pinMode(BUTTON_PIN_4, INPUT_PULLUP);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(5000);
    Serial.print(".");
  }
  Serial.println("Verbunden mit WiFi");
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Adresse: ");
  Serial.println(ip);

  server.begin();
  Serial.println("Webserver gestartet");

  sensors.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  delay(2000);
  display.clearDisplay();

  ph.begin();

  if (!SD.begin(SD_CARD_CS_PIN)) {
    Serial.println("SD-Karteninitialisierung fehlgeschlagen!");
    return;
  }
  Serial.println("SD-Karte ist bereit.");
}

void loop() {
  unsigned long currentTime = millis();
  updateSensorValues();
  updateCorrectionCounts();  // Zähler aktualisieren
  displaySensorValues();
  controlPumpsWithButtons();

  if (!isFirstPHValueMeasured) {
    firstPHValue = averagePH;
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

void updateCorrectionCounts() {
  unsigned long oneHourAgo = millis() - 3600000;
  minusCorrectionCount = 0;
  plusCorrectionCount = 0;

  for (int i = 0; i < correctionIndex; i++) {
    if (correctionTimes[i] > oneHourAgo) {
      if (i % 2 == 0) { // Gerade Indizes für pH-Minus
        minusCorrectionCount++;
      } else { // Ungerade Indizes für pH-Plus
        plusCorrectionCount++;
      }
    }
  }
}

void displaySensorValues() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(currentTemperature);
  display.print(" C");

  display.setCursor(0, 10);
  display.print("EC: ");
  display.print(globalEcValue, 2);

  display.setCursor(0, 20);
  display.print("pH: ");
  display.print(averagePH);

  display.setCursor(0, 30);
  display.print("pH-Set: ");
  display.print(setPointPH);

  display.setCursor(0, 40);
  display.print("pH-Minus: Last ");
  display.print(minusCorrectionCount);
  display.print(" hr");

  display.setCursor(0, 50);
  display.print("pH-Plus: Last ");
  display.print(plusCorrectionCount);
  display.print(" hr");

  display.display();
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
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temperature);
  display.print(" C");

  display.setCursor(0, 10);
  display.print("EC: ");
  display.print(ecValue, 2);

  display.display();
}

float measurePH() {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  float voltage = analogRead(PH_SENSOR_PIN)/1024.0*5000;
  return ph.readPH(voltage, temperature);
}

void controlPumps(float pHValue) {
  unsigned long currentMillis = millis();

  // Pumpe 1 an D8 senkt den pH-Wert, wenn er über dem Sollwert plus Schwellenwert liegt
  if (pHValue > setPointPH + PH_THRESHOLD && !isPump1Attached) {
    pump1.attach(8); // Pumpe 1 für pH-Minus
    pump1.write(170); // Aktivierung der Pumpe
    delay(100);
    pump1.write(90); // Deaktivierung der Pumpe
    pump1.detach();
    correctionTimes[correctionIndex++] = currentMillis; // Speichern des Zeitstempels
    if (correctionIndex >= 120) correctionIndex = 0; // Array-Wrap-around
    minusCorrectionCount++; // Zählung der pH-Minus Korrekturen
  } 

  // Pumpe 2 an D9 erhöht den pH-Wert, wenn er unter dem Sollwert minus Schwellenwert liegt
  if (pHValue < setPointPH - PH_THRESHOLD && !isPump2Attached) {
    pump2.attach(9); // Pumpe 2 für pH-Plus
    pump2.write(10); // Aktivierung der Pumpe
    delay(100);
    pump2.write(90); // Deaktivierung der Pumpe
    pump2.detach();
    correctionTimes[correctionIndex++] = currentMillis; // Speichern des Zeitstempels
    if (correctionIndex >= 120) correctionIndex = 0; // Array-Wrap-around
    plusCorrectionCount++; // Zählung der pH-Plus Korrekturen
  }
}



void controlPumpsWithButtons() {
  int buttonState1 = digitalRead(BUTTON_PIN_1);
  int buttonState2 = digitalRead(BUTTON_PIN_2);
  int buttonState3 = digitalRead(BUTTON_PIN_3);  // Taste für pH erhöhen
  int buttonState4 = digitalRead(BUTTON_PIN_4);  // Taste für pH verringern


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

   if (buttonState3 == LOW) {
    setPointPH += 0.05;  // Erhöht den pH-Sollwert um 0.5
    Serial.print("pH-Sollwert erhöht auf: ");
    Serial.println(setPointPH);
    delay(500); // Entprellung und um versehentliche mehrfache Änderungen zu vermeiden
  }

  if (buttonState4 == LOW) {
    setPointPH -= 0.05;  // Verringert den pH-Sollwert um 0.5
    Serial.print("pH-Sollwert verringert auf: ");
    Serial.println(setPointPH);
    delay(500); // Entprellung und um versehentliche mehrfache Änderungen zu vermeiden
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
