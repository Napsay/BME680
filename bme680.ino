#include <AsyncPrinter.h>
#include <async_config.h>
#include <DebugPrintMacros.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncTCPbuffer.h>
#include <SyncClient.h>
#include <tcp_axtls.h>


#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define WIFI_SSID "TP-Link_BB0E"
#define WIFI_PASSWORD "80648628"


#define MQTT_HOST IPAddress(192,168,1,106)
#define MQTT_PORT 1883

#define MQTT_PUB_TEMP "esp/bme680/temperature"
#define MQTT_PUB_HUM  "esp/bme680/humidity"
#define MQTT_PUB_GAS  "esp/bme680/gas"
#define MQTT_PUB_ZAGADJENJE "esp/bme680/zagadjenje"
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("BME680 async test"));

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }
    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials("pilab", "pilab01");
  connectToWifi();
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); 
}

void loop() {
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  Serial.print(F("Temperature = "));
  float temperature = bme.temperature;
  float prviGrafikon = 0;
  float drugiGrafikon = 0;
  float treciGrafikon = 0;
  if(temperature >= 21)
  {
  prviGrafikon = map(temperature, 21, 100, 0, 10);
  }
  else
  {
  prviGrafikon = map(temperature, 21, -20, 0, 10);
  }
  Serial.print(temperature);
  Serial.println(F(" *C"));
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temperature).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperature);

  Serial.print(F("Humidity = "));
  float humidity = bme.humidity;
  if(humidity>=40)
  {
  drugiGrafikon = map(humidity, 40, 100, 0, 10);
  }
  else
  {
  drugiGrafikon = map(humidity, 40, 0, 0, 10);
  }
  Serial.print(humidity);
  Serial.println(F(" %"));
  uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(humidity).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidity);

  Serial.print(F("Gas = "));
  
  float gas = bme.gas_resistance / 1000.0;
  if(gas<50)
  {
  treciGrafikon = map(gas, 50, 5, 0, 80);
  }
  else
  {
  treciGrafikon = 0;
  }
  uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_GAS, 1, true, String(gas).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_GAS, packetIdPub4);
    Serial.printf("Message: %.2f \n", gas);
  Serial.print(gas);
  Serial.println(F(" KOhms"));
  float ukupno = prviGrafikon + drugiGrafikon + treciGrafikon;
  Serial.print("Prvi grafikon: ");
  Serial.println(prviGrafikon);
  Serial.print("Drugi grafikon: ");
  Serial.println(drugiGrafikon);
  Serial.print("Treci grafikon: ");
  Serial.println(treciGrafikon);
  Serial.print("Ukupno: ");
  Serial.println(ukupno);
  uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_ZAGADJENJE, 1, true, String(ukupno).c_str());
  
  delay(2000);
}
