/**
 * TEIDESAT Dummies - Transmitter Firmware
 * Runs on an ESP32 WROOM 32
 * Core 1 continuously polls the API server for the current firmware state
 * Core 0 handles the actual data fetching and optical transmission when in "Sending" state
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <mutex>
#include "secrets.h"

//==============================================================================

#define LIGHT_PIN (18)      // ESP32 Pin to send light pulses
#define DEBUG_LED_PIN (19)  // ESP32 Pin for debugging

#define SAMPLE_RATE (1000000)  // Base sample rate: 1 MHz

//==============================================================================

const String wifiSsid = WIFI_SSID;
const String wifiPassword = WIFI_PASSWORD;
const String apiServerBaseUrl = API_SERVER_BASE_URL;  // Change to server's IP address

// Endpoints for polling
const String messageDataUrl = apiServerBaseUrl + "/get_message_data";
const String blinkingFrequencyUrl = apiServerBaseUrl + "/get_blinking_frequency";
const String firmwareStateUrl = apiServerBaseUrl + "/firmware_state";

// Mutex to prevent Wi-Fi hardware collisions between core 0 and core 1
std::mutex HTTP_CLIENT_IN_USE;

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
  Serial.begin(115200);
  disableCore0WDT();

  // Quick LED test: blink LIGHT_PIN 3 times to verify hardware wiring and pin
  for (int i = 0; i < 3; ++i) {
    digitalWrite(LIGHT_PIN, HIGH);
    delay(300);
    digitalWrite(LIGHT_PIN, LOW);
    delay(300);
  }

  setupWiFi();
}

// Core 1 loop
void loop() {
  WiFiClient wifiClient;
  HTTPClient httpClient;

  std::lock_guard<std::mutex> guard(HTTP_CLIENT_IN_USE);
  const String currentState = getRequest(wifiClient, httpClient, firmwareStateUrl);

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
  delay(2000); // Wait between state requests to avoid flooding the server
}

//==============================================================================

// Core 0 loop
void sendLoop(void* parameters) {
  WiFiClient wifiClient;
  HTTPClient httpClient;
  String messageData;
  String blinkingFrequency;
  
  while (true) {
    vTaskDelay(1);
    
    std::lock_guard<std::mutex> guard(HTTP_CLIENT_IN_USE);
    
    messageData = getRequest(wifiClient, httpClient, messageDataUrl);
    if (messageData == "") {
      break;
    }

    blinkingFrequency = getRequest(wifiClient, httpClient, blinkingFrequencyUrl);
    if (blinkingFrequency == "") {
      break;
    }
    
    sendMessage(messageData, blinkingFrequency.toFloat());
  }
  
  // Cleanup if loop breaks
  digitalWrite(LIGHT_PIN, LOW);
  SEND_LOOP_HANDLE = NULL;
  vTaskDelete(NULL);
}


void setupWiFi() {
  WiFi.persistent(false);                 // Don't save WiFi credentials to flash
  WiFi.mode(WIFI_STA);                    // Set WiFi to station mode
  WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
  WiFi.disconnect(true, true);            // Disconnect from any previous WiFi connections and erase credentials
  delay(250);
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("  ...Status code: ");
    Serial.println(WiFi.status());
    delay(500);
  }

  Serial.println("WiFi connection established.");
}

bool ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.println("WiFi disconnected, reconnecting...");
  setupWiFi();
  return WiFi.status() == WL_CONNECTED;
}

void sendMessage(const String &messageData, const float &blinkingFrequency) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending binary data payload...");

  const auto bitWaitTime = static_cast<unsigned long>(SAMPLE_RATE / blinkingFrequency); 

  unsigned long nextBitTime = micros(); 

  for (const auto messageByte : messageData) {
    digitalWrite(LIGHT_PIN, messageByte == '1' ? HIGH : LOW);
    
    nextBitTime += bitWaitTime;
    
    while (micros() < nextBitTime) {
    }
  }

  digitalWrite(LIGHT_PIN, LOW);
  
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