#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <mutex>

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

std::mutex HTTP_CLIENT_IN_USE;

TaskHandle_t SEND_LOOP_HANDLE = NULL;

//==============================================================================

void sendLoop(void * parameters);
void sendMessage(const String &messageData, const float &blinkingFrequency);
String getRequest(WiFiClient& wifiClient, HTTPClient& httpClient, const String& targetUrl);
void setupWiFi();

//==============================================================================

void setup() {
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(DEBUG_LED_PIN, OUTPUT);
  Serial.begin(9600);
  disableCore0WDT();

  setupWiFi();
}

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
  sleep(2); // Wait between state requests
}

//==============================================================================

void sendLoop(void* parameters) {
  WiFiClient wifiClient;
  HTTPClient httpClient;
  String messageData;
  String blinkingFrequency;
  while (true) {
    { // Scope for the lock guard
      vTaskDelay(1);
      std::lock_guard<std::mutex> guard(HTTP_CLIENT_IN_USE);
      messageData = getRequest(wifiClient, httpClient, messageDataUrl);
      Serial.println("Message data: " + messageData);

      if (messageData == "") {
        Serial.println("Error getting message data");
        break;
      }

      blinkingFrequency = getRequest(wifiClient, httpClient, blinkingFrequencyUrl);
      Serial.println("Blinking frequency: " + blinkingFrequency);

      if (blinkingFrequency == "") {
        Serial.println("Error getting blinking frequency");
        break;
      }
    }
    sendMessage(messageData, blinkingFrequency.toFloat());
  }
  SEND_LOOP_HANDLE = NULL;
  vTaskDelete(NULL);
  digitalWrite(LIGHT_PIN, LOW);
}

void setupWiFi() {
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Connecting to WiFi...");

  while (WiFiClass::status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  Serial.println("WiFi connection established.");
}

void sendMessage(const String &messageData, const float &blinkingFrequency) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending message data");

  const auto bitWaitTime = static_cast<unsigned long>(SAMPLE_RATE / blinkingFrequency);  // Time in microseconds for each bit
  //unsigned long startTime = micros();

  for (const auto messageByte : messageData) {
    Serial.println("Current char: " + String(messageByte));
    for (int i{7}; i >= 0; --i) {
      int currentBit = (messageByte >> i) & 1;
      Serial.println("Current bit: " + String(currentBit));
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

  if (httpClient.begin(wifiClient, targetUrl)) {
    Serial.println("URL initialized");
  }

  const int responseCode = httpClient.GET();

  if (responseCode != HTTP_CODE_OK) {
    Serial.printf(
        "[HTTP] GET... failed, error: %s\n",
        HTTPClient::errorToString(responseCode).c_str()
    );

    return "";
  }

  Serial.printf("[HTTP] GET... code: %d", responseCode);

  String responsePayload = httpClient.getString();
  Serial.println("Received payload: " + responsePayload);

  return responsePayload;
}