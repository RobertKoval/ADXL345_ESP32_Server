#include <ESP32Time.h>

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

#include <WebSockets.h>
#include <WebSocketsServer.h>

#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include "config.h"

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(5, 3, 4, 2, 12345);  //Adafruit_ADXL345_Unified(12345);

WebSocketsServer webSocket = WebSocketsServer(81);

const int capacity = JSON_OBJECT_SIZE(4);
StaticJsonDocument<capacity> doc;

ESP32Time rtc(0);

void displaySensorDetails(void) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


char output[128];
sensors_event_t event;

uint8_t lastConnectedClient = -1;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      lastConnectedClient = -1;
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        lastConnectedClient = num;
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

void setup(void) {
  Serial.begin(115200);
  Serial.println("Accelerometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");

    while (1)
      ;
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Display additional settings (outside the scope of sensor_t) */
  Serial.println("");


  // WIfi
  WiFi.begin(MY_WIFI_SSID, MY_WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop(void) {
  if (lastConnectedClient != -1 && webSocket.clientIsConnected(lastConnectedClient)) {
    // send message to client
    accel.getEvent(&event);
    doc["timestamp"] = rtc.getEpoch();
    doc["axisX"] = event.acceleration.x;
    doc["axisY"] = event.acceleration.y;
    doc["axisZ"] = event.acceleration.z;
    serializeJson(doc, output);
    webSocket.broadcastTXT(output, strlen(output));
  }

  webSocket.loop();
}