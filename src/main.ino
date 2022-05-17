/*
MIT License

Copyright (c) 2022 Kai Anter

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_ADT7410.h>

// Target: Send MQTT Message every minute
// With oversampling: Measurement every 0.5 seconds

#define READINGS 60           // one minute: 120 readings à 500ms
#define READINGMILLISEC 1000
#define BUFFER_SIZE 2048
#define SENSOR_ID 0
// #define DEBUG
#include "secrets.h"
// Temperature Sensor
Adafruit_ADT7410 tempsensor = Adafruit_ADT7410();

// Timed readings
unsigned long lastReading = 0;
unsigned long lastAverage;
float readings[READINGS];
int readingCount = 0;

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

const char MSG_SEPARATOR = ' ';

struct avgReading
{
  unsigned int id;
  float avgTemp;
  unsigned long timestamp;
};

avgReading readingBuf[BUFFER_SIZE];
int bufferCount = 0;

bool canConnectToBroker = false;

// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate
//    flashed in the WiFi module.

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "your.broker.domain.or.ip";
int port = 1883;
const char topic[] = "/your/topic";
const char MQTT_USER[] = "mqtt_user";
const char MQTT_PASS[] = "mqtt_pass";

const long interval = 5000;
unsigned long previousMillis = 0;

unsigned int count = 0;

void setup()
{
  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial)
  {
    ;
  }
#endif

  // Temp sensor
  if (!tempsensor.begin())
  {
    Serial.println("Couldn't find ADT7410!");
    while (1)
      ;
  }

  // connect to wifi
  Serial.print("Attempting to connect to Wifi...");
  while (WiFi.begin(ssid, pass) != WL_CONNECTED)
  {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println("connected");

  // You can provide a unique client ID, if not set the library uses Arduino-millis()
  // Each client must have a unique client ID
  // mqttClient.setId("clientId");

  Serial.print("Connecting to broker ");
  Serial.println(broker);

  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASS);
  if (!mqttClient.connect(broker, port))
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1)
      ;
  }
  canConnectToBroker = true;

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
}

bool areServicesConnected()
{
  bool wifi = wifiClient.connected();
  bool mqtt = mqttClient.connected();
  bool connected = wifi && mqtt;

  if (connected)
  {
    return true;
  }
  else
  {
    if (!wifi)
      Serial.println("error: not connected to wifi");

    if (!mqtt)
      Serial.println("error: not connected to mqtt server");

    return false;
  }
}

void checkAndReconnectServices()
{
  if (!canConnectToBroker)
  {
    if (!wifiClient.connected())
    {
      int attempts = 0;
      while ((WiFi.begin(ssid, pass) != WL_CONNECTED) && attempts < 3)
      {
        Serial.println("connection to wifi lost, trying to reconnect...");
        delay(1000);
        attempts++;
      }
    }

    Serial.println("connection mqtt broker lost, trying to reconnect...");
    if (!mqttClient.connect(broker, port))
    {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
    }
    else
    {
      canConnectToBroker = true;
    }

    delay(1000);
  }
}

void loop()
{
  checkAndReconnectServices();

  trySendBuffer();

  mqttClient.poll(); // for keep alive

  doReading();
}

void doReading()
{
  unsigned long currentMillis = millis();

  if ((currentMillis - lastReading) > READINGMILLISEC)
  {
    float c = tempsensor.readTempC();
    // Serial.print("Temp: ");
    // Serial.print(c);
    // Serial.print("*C\n");
    lastReading = currentMillis;
    readings[readingCount] = c;
    readingCount++;
  }

  if (readingCount == READINGS)
  {
    float average = getAverage();
    readingCount = 0;
    sendTemperature(average, WiFi.getTime());
  }
}

void trySendBuffer()
{
  if (bufferCount == 0)
    return;

  for (int i = bufferCount; i > 0; i--)
  {
    avgReading msg = readingBuf[i - 1];

    if (!sendMQTTMessage(msg.id, msg.avgTemp, msg.timestamp))
    {
      Serial.println("error while uploading message from buffer");
      delay(1000);
      return;
    }

    bufferCount -= 1;
    Serial.println("uploaded value from buffer");
  }
}

int sendMQTTMessage(int id, float avgTemp, unsigned long unix_time)
{
  int success = mqttClient.beginMessage(topic);
  if (!success)
  {
    Serial.println("an error occurred while beginning the mqtt message");
    canConnectToBroker = false;
    return 0;
  }

  mqttClient.print(SENSOR_ID);
  mqttClient.print(MSG_SEPARATOR);
  mqttClient.print(id);
  mqttClient.print(MSG_SEPARATOR);
  mqttClient.print(avgTemp);
  mqttClient.print(MSG_SEPARATOR);
  mqttClient.print(unix_time);

  Serial.print("Sending temperature nr");
  Serial.print(id);
  Serial.print(", ");
  Serial.println(avgTemp);

  success = mqttClient.endMessage();
  if (success)
    blinkShort();
  else
    canConnectToBroker = false;

  return success;
}

void saveToBuffer(float avgTemp, unsigned long unix_time)
{
  if (bufferCount >= BUFFER_SIZE)
  {
    Serial.print("max buffer count reached, information is lost!\n");
    return;
  }

  readingBuf[bufferCount] = {
    id : count,
    avgTemp : avgTemp,
    timestamp : unix_time
  };
  bufferCount += 1;
  Serial.print("value saved into buffer, current count: ");
  Serial.print(bufferCount);
  Serial.println("");
}

void sendTemperature(float avgTemp, unsigned long unix_time)
{
  if (!sendMQTTMessage(count, avgTemp, unix_time))
  {
    Serial.println("failed to send mqtt message");
    saveToBuffer(avgTemp, unix_time);
  }

  count++;
}

float getAverage()
{
  float sum = 0;
  for (int i = 0; i < READINGS; i++)
  {
    sum += readings[i];
  }
  return sum / READINGS;
}

void blinkShort()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}