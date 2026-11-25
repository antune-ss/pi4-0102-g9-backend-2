/*
- CO2NNECT | ESP32-S3 + SCD30 + 2 OLED SSD1306 (128x64)
- I2C principal Wire: SDA GPIO4, SCL GPIO5 -> SCD30 (0x61) e OLED1 CO2 (0x3C)
- I2C secundario Wire1: SDA GPIO6, SCL GPIO7 -> OLED2 temp/umidade (0x3C)
- Bibliotecas: SparkFun SCD30 Arduino Library, PubSubClient,
- Adafruit GFX Library, Adafruit SSD1306.  
- Copie config.example.h para config.h na mesma pasta deste .ino.
 */
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <math.h>
#include "config.h"

constexpr uint8_t MAIN_SDA_PIN = 4;
constexpr uint8_t MAIN_SCL_PIN = 5;
constexpr uint8_t SECOND_SDA_PIN = 6;
constexpr uint8_t SECOND_SCL_PIN = 7;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr uint32_t SENSOR_RETRY_MS = 5000;
constexpr uint32_t SENSOR_POLL_MS = 2000;
constexpr uint32_t SENSOR_STALE_MS = 15000;
constexpr uint32_t MQTT_SEND_MS = 5000;
constexpr uint32_t WIFI_RETRY_MS = 10000;
constexpr uint32_t MQTT_RETRY_MS = 5000;
constexpr uint32_t OLED_REFRESH_MS = 1000;
constexpr uint32_t SERIAL_BAUD = 115200;

TwoWire secondaryI2C = TwoWire(1);
SCD30 scd30;
Adafruit_SSD1306 oledCO2(128, 64, &Wire, -1);
Adafruit_SSD1306 oledClimate(128, 64, &secondaryI2C, -1);

// CORREÇÃO: Utilizando WiFiClientSecure para suportar a porta 8883 (TLS/EMQX Cloud)
WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);

bool sensorReady = false;
bool oledCO2Ready = false;
bool oledClimateReady = false;
bool hasReading = false;
uint16_t lastCO2 = 0;
float lastTemp = NAN;
float lastHumidity = NAN;
uint32_t lastSensorAttempt = 0;
uint32_t lastSensorPoll = 0;
uint32_t lastReadingMs = 0;
uint32_t lastWifiAttempt = 0;
uint32_t lastMqttAttempt = 0;
uint32_t lastPublishMs = 0;
uint32_t lastOledMs = 0;
String telemetryTopic;
String statusTopic;
String mqttClientId;

bool readingFresh(uint32_t now) {
  return hasReading && (now - lastReadingMs <= SENSOR_STALE_MS);
}

bool timestampUtc(char* out, size_t len) {
  time_t current = time(nullptr);
  if (current <= 1700000000) return false;
  struct tm utc;
  if (gmtime_r(&current, &utc) == nullptr) return false;
  return strftime(out, len, "%Y-%m-%dT%H:%M:%SZ", &utc) > 0;
}

void connectWifiIfNeeded(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) return;
  if (lastWifiAttempt != 0 && now - lastWifiAttempt < WIFI_RETRY_MS) return;
  lastWifiAttempt = now;
  Serial.println("[WiFi] Tentando conectar...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectMqttIfNeeded(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (lastMqttAttempt != 0 && now - lastMqttAttempt < MQTT_RETRY_MS) return;
  lastMqttAttempt = now;

  // Aguarda sincronização de relógio NTP antes de tentar o handshake TLS
  time_t nowSecs = time(nullptr);
  if (nowSecs < 1700000000) {
    Serial.println("[MQTT] Aguardando sincronizacao de horario (NTP) para validar o certificado TLS...");
    return;
  }

  Serial.println("[MQTT] Conectando ao broker via TLS (Porta 8883)...");
  bool ok = mqtt.connect(mqttClientId.c_str(), MQTT_USER, MQTT_PASSWORD, statusTopic.c_str(), 1, true, "0");
  if (!ok) {
    Serial.printf("[MQTT] Falha ao conectar: rc=%d\n", mqtt.state());
    return;
  }
  mqtt.publish(statusTopic.c_str(), "1", true);
  Serial.println("[MQTT] Conectado com sucesso; status online publicado.");
}

void initSensorIfNeeded(uint32_t now) {
  if (sensorReady) return;
  if (lastSensorAttempt != 0 && now - lastSensorAttempt < SENSOR_RETRY_MS) return;
  lastSensorAttempt = now;
  sensorReady = scd30.begin(Wire);
  if (sensorReady) {
    scd30.setMeasurementInterval(2);
    Serial.println("[SCD30] Inicializado no barramento Wire (GPIO4/5).");
  } else {
    Serial.println("[SCD30] Nao encontrado; nova tentativa em 5 s.");
  }
}

void pollSensor(uint32_t now) {
  if (!sensorReady || now - lastSensorPoll < SENSOR_POLL_MS) return;
  lastSensorPoll = now;
  if (!scd30.dataAvailable()) {
    if (hasReading && now - lastReadingMs > SENSOR_STALE_MS) {
      hasReading = false;
      sensorReady = false;
      Serial.println("[SCD30] Leitura expirada; reinicializando sensor.");
    }
    return;
  }
  uint16_t co2 = scd30.getCO2();
  float temp = scd30.getTemperature();
  float humidity = scd30.getHumidity();
  if (co2 == 0 || !isfinite(temp) || !isfinite(humidity) ||
      temp < -40 || temp > 100 || humidity < 0 || humidity > 100) {
    Serial.println("[SCD30] Leitura invalida descartada.");
    return;
  }
  lastCO2 = co2;
  lastTemp = temp;
  lastHumidity = humidity;
  hasReading = true;
  lastReadingMs = now;
  Serial.printf("[SCD30] CO2=%u ppm | T=%.1f C | UR=%.1f %%\n", co2, temp, humidity);
}

void publishReading(uint32_t now) {
  if (now - lastPublishMs < MQTT_SEND_MS) return;
  lastPublishMs = now;
  if (!readingFresh(now) || !mqtt.connected()) return;
  char timestamp[25];
  if (!timestampUtc(timestamp, sizeof(timestamp))) {
    Serial.println("[MQTT] Aguardando horario NTP.");
    return;
  }
  char json[256];
  int n = snprintf(json, sizeof(json),
      "{\"deviceId\":\"%s\",\"co2\":%u,\"temperature\":%.2f,\"humidity\":%.2f,\"measuredAt\":\"%s\"}",
      DEVICE_ID, lastCO2, lastTemp, lastHumidity, timestamp);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(json)) {
    Serial.println("[MQTT] Payload excedeu buffer.");
    return;
  }
  if (mqtt.publish(telemetryTopic.c_str(), json)) {
    Serial.printf("[MQTT] %s -> %s\n", telemetryTopic.c_str(), json);
  } else {
    Serial.println("[MQTT] Falha na publicacao.");
  }
}

void refreshOleds(uint32_t now) {
  if (now - lastOledMs < OLED_REFRESH_MS) return;
  lastOledMs = now;
  const bool fresh = readingFresh(now);

  if (oledCO2Ready) {
    oledCO2.clearDisplay();
    oledCO2.setTextColor(SSD1306_WHITE);
    oledCO2.setTextSize(1);
    oledCO2.setCursor(0, 0);
    oledCO2.println("CO2NNECT | CO2");
    oledCO2.drawLine(0, 12, 127, 12, SSD1306_WHITE);
    if (fresh) {
      oledCO2.setTextSize(3);
      oledCO2.setCursor(5, 22);
      oledCO2.print(lastCO2);
      oledCO2.setTextSize(1);
      oledCO2.setCursor(90, 49);
      oledCO2.print("ppm");
    } else {
      oledCO2.setCursor(10, 30);
      oledCO2.println("SEM LEITURA");
    }
    oledCO2.display();
  }

  if (oledClimateReady) {
    oledClimate.clearDisplay();
    oledClimate.setTextColor(SSD1306_WHITE);
    oledClimate.setTextSize(1);
    oledClimate.setCursor(0, 0);
    oledClimate.println("CO2NNECT | CLIMA");
    oledClimate.drawLine(0, 12, 127, 12, SSD1306_WHITE);
    if (fresh) {
      oledClimate.setCursor(0, 20);
      oledClimate.print("TEMP:");
      oledClimate.setTextSize(2);
      oledClimate.setCursor(0, 30);
      oledClimate.print(lastTemp, 1);
      oledClimate.print(" C");
      oledClimate.setTextSize(1);
      oledClimate.setCursor(0, 51);
      oledClimate.print("UMIDADE: ");
      oledClimate.print(lastHumidity, 1);
      oledClimate.print("%");
    } else {
      oledClimate.setCursor(10, 30);
      oledClimate.println("SEM LEITURA");
    }
    oledClimate.display();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000); // Aguarda a serial estabilizar para evitar travamentos de boot
  
  Wire.begin(MAIN_SDA_PIN, MAIN_SCL_PIN);
  Wire.setClock(100000);
  secondaryI2C.begin(SECOND_SDA_PIN, SECOND_SCL_PIN);
  secondaryI2C.setClock(100000);

  oledCO2Ready = oledCO2.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oledClimateReady = oledClimate.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledCO2Ready) Serial.println("[OLED1] Nao encontrado em GPIO4/5.");
  if (!oledClimateReady) Serial.println("[OLED2] Nao encontrado em GPIO6/7.");

  telemetryTopic = String("co2nect/") + DEVICE_ID + "/telemetry";
  statusTopic = String("co2nect/") + DEVICE_ID + "/status";
  mqttClientId = String("co2nnect-") + DEVICE_ID;
  
  // Configura o cliente seguro com o certificado CA definido no config.h
  secureClient.setCACert(MQTT_ROOT_CA);
  
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(20);
  mqtt.setSocketTimeout(5);
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  
  initSensorIfNeeded(millis());
  refreshOleds(millis());
  Serial.println("[CO2NNECT] Firmware iniciado com sucesso.");
}

void loop() {
  uint32_t now = millis();
  initSensorIfNeeded(now);
  pollSensor(now);
  connectWifiIfNeeded(now);
  connectMqttIfNeeded(now);
  if (mqtt.connected()) mqtt.loop();
  publishReading(now);
  refreshOleds(now);
  delay(5);
}