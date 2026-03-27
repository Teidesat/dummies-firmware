#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

//==============================================================================

#define LIGHT_PIN (18)  // ESP32 Pin to send light pulses
#define DEBUG_LED_PIN (19)  // ESP32 Pin for debugging

#define SAMPLE_RATE (1000000)  // 1 MHz

//==============================================================================

const String wifiSsid = "transmitter-dummy";
const String wifiPassword = "transmitter-dummy";

const String apiServerBaseUrl = "http://10.42.0.1:5000";  // Change to server's IP address

const String messageDataUrl = apiServerBaseUrl + "/get_message_data";
const String blinkingFrequencyUrl = apiServerBaseUrl + "/get_blinking_frequency";
const String firmwareStateUrl = apiServerBaseUrl + "/firmware_state";

SemaphoreHandle_t HTTP_CLIENT_IN_USE = nullptr;

TaskHandle_t SEND_LOOP_HANDLE = NULL;

//==============================================================================

void sendLoop(void * parameters);
void sendMessage(const String &messageData, const float &blinkingFrequency);
String getRequest(WiFiClient& wifiClient, HTTPClient& httpClient, const String& targetUrl);
void setupWiFi();
bool ensureWiFiConnected();

//==============================================================================

void setup() {
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(DEBUG_LED_PIN, OUTPUT);
  Serial.begin(9600);
  disableCore0WDT();

  HTTP_CLIENT_IN_USE = xSemaphoreCreateMutex();
  if (HTTP_CLIENT_IN_USE == nullptr) {
    Serial.println("Failed to create HTTP mutex");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  setupWiFi();
}

void loop() {
  WiFiClient wifiClient;
  HTTPClient httpClient;

  String currentState;
  if (xSemaphoreTake(HTTP_CLIENT_IN_USE, portMAX_DELAY) == pdTRUE) {
    currentState = getRequest(wifiClient, httpClient, firmwareStateUrl);
    xSemaphoreGive(HTTP_CLIENT_IN_USE);
  }

  if (currentState == "Sending" && SEND_LOOP_HANDLE == NULL) {
    Serial.println("Sending");
    xTaskCreatePinnedToCore(sendLoop, "sendLoop", 8192, nullptr, 1, &SEND_LOOP_HANDLE, 0);
  } else if (currentState == "Idle" && SEND_LOOP_HANDLE != NULL) {
    Serial.println("Idle");
    vTaskDelete(SEND_LOOP_HANDLE);
    SEND_LOOP_HANDLE = NULL;
    digitalWrite(LIGHT_PIN, LOW);
  } else {
    Serial.print("Current state: ");
    Serial.println(currentState);
  }
  vTaskDelay(pdMS_TO_TICKS(2000)); // Wait between state requests
}

//==============================================================================

void sendLoop(void* parameters) {
  WiFiClient wifiClient;
  HTTPClient httpClient;
  String messageData;
  String blinkingFrequency;
  while (true) {
    {
      vTaskDelay(pdMS_TO_TICKS(1));
      if (xSemaphoreTake(HTTP_CLIENT_IN_USE, portMAX_DELAY) != pdTRUE) {
        continue;
      }

      messageData = getRequest(wifiClient, httpClient, messageDataUrl);
      Serial.println("Message data: " + messageData);

      if (messageData == "") {
        Serial.println("Error getting message data");
        xSemaphoreGive(HTTP_CLIENT_IN_USE);
        break;
      }

      blinkingFrequency = getRequest(wifiClient, httpClient, blinkingFrequencyUrl);
      Serial.println("Blinking frequency: " + blinkingFrequency);

      if (blinkingFrequency == "") {
        Serial.println("Error getting blinking frequency");
        xSemaphoreGive(HTTP_CLIENT_IN_USE);
        break;
      }

      xSemaphoreGive(HTTP_CLIENT_IN_USE);
    }
    sendMessage(messageData, blinkingFrequency.toFloat());
  }
  SEND_LOOP_HANDLE = NULL;
  vTaskDelete(NULL);
  digitalWrite(LIGHT_PIN, LOW);
}

void setupWiFi() {
  WiFi.persistent(false);                 // Don't save WiFi credentials to flash
  WiFi.mode(WIFI_STA);                    // Set WiFi to station mode
  WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
  WiFi.disconnect(true, true);            // Disconnect from any previous WiFi connections and erase credentials
  delay(250);
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.println("Connecting to WiFi...");

  while (WiFiClass::status() != WL_CONNECTED) {
    Serial.print("  ...Status code: ");
    Serial.println(WiFiClass::status());
    delay(500);
  }

  Serial.println("WiFi connection established.");
}

bool ensureWiFiConnected() {
  if (WiFiClass::status() == WL_CONNECTED) {
    return true;
  }

  Serial.println("WiFi disconnected, reconnecting...");
  setupWiFi();
  return WiFiClass::status() == WL_CONNECTED;
}

void sendMessage(const String &messageData, const float &blinkingFrequency) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending message data: " + messageData);

  const auto bitWaitTime = static_cast<unsigned long>(SAMPLE_RATE / blinkingFrequency);  // Time in microseconds for each bit
  //unsigned long startTime = micros();

  for (const auto messageByte : messageData) {
    // Serial.println("Current char: " + String(messageByte));
    for (int i{7}; i >= 0; --i) {
      int currentBit = (messageByte >> i) & 1;
      // Serial.println("Current bit: " + String(currentBit));
      digitalWrite(LIGHT_PIN, currentBit == 1 ? HIGH : LOW);  // Send the bit
      /*
      while ((micros() - startTime) < bitWaitTime) {
        // Wait for the bit time to pass
      }
      */
      delayMicroseconds(bitWaitTime);
    //startTime += bitWaitTime;
    }
  }

  Serial.println("Data sent");
  digitalWrite(DEBUG_LED_PIN, LOW);
}


String getRequest(WiFiClient& wifiClient, HTTPClient& httpClient, const String& targetUrl) {
  Serial.println("Sending http request");

  if (!ensureWiFiConnected()) {
    return "";
  }

  httpClient.setReuse(false);

  if (!httpClient.begin(wifiClient, targetUrl)) {
    Serial.println("Failed to initialize URL");
    wifiClient.stop();
    return "";
  }

  Serial.println("URL initialized");

  const int responseCode = httpClient.GET();

  if (responseCode != HTTP_CODE_OK) {
    Serial.printf(
        "[HTTP] GET... failed, error: %s\n",
        HTTPClient::errorToString(responseCode).c_str()
    );

    httpClient.end();
    wifiClient.stop();

    return "";
  }

  Serial.printf("[HTTP] GET... code: %d", responseCode);

  String responsePayload = httpClient.getString();
  httpClient.end();
  Serial.println("Received payload: " + responsePayload);

  return responsePayload;
}
