#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>

//==============================================================================

#define LIGHT_PIN (18)  // ESP32 Pin to send light pulses
#define DEBUG_LED_PIN (19)  // ESP32 Pin for debugging

#define SAMPLE_RATE (1000000)  // 1 MHz

//==============================================================================

const String wifiSsid = "xxxxxx";
const String wifiPassword = "xxxxxx";

const String apiServerBaseUrl = "http://xxxxxx:5000";  // Change to server's IP address

const String sendModeUrl = apiServerBaseUrl + "/get_send_mode";
const String blinkingFrequencyUrl = apiServerBaseUrl + "/get_blinking_frequency";
const String messageDataUrl = apiServerBaseUrl + "/get_message_data";

//==============================================================================

void sendString(float blinkingFrequency, const String& messageData);
void sendBinary(float blinkingFrequency, const uint8_t* messageData, unsigned int dataSize);
String getRequest(WiFiClient& wifiClient, HTTPClient& httpClient, const String& targetUrl);
void setupWiFi();

//==============================================================================

void setup() {
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(DEBUG_LED_PIN, OUTPUT);

  Serial.begin(9600);

  setupWiFi();
}

void loop() {
  WiFiClient wifiClient;
  HTTPClient httpClient;

  const String sendMode = getRequest(wifiClient, httpClient, sendModeUrl);
  Serial.println("Send mode: " + sendMode);

  if (sendMode == "Error") {
    Serial.println("Error getting send mode");
    return;
  }

  if (sendMode == "send_string") {
    const String blinkingFrequency = getRequest(wifiClient, httpClient, blinkingFrequencyUrl);
    Serial.println("Frequency: " + blinkingFrequency);

    if (blinkingFrequency == "Error") {
      Serial.println("Error getting blinking frequency");
      return;
    }

    const String messageData = getRequest(wifiClient, httpClient, messageDataUrl);
    Serial.println("Data: " + messageData);

    if (messageData == "Error") {
      Serial.println("Error getting message data");
      return;
    }

    sendString(blinkingFrequency.toFloat(), messageData);
  }

  if (sendMode == "send_binary") {
    const String blinkingFrequency = getRequest(wifiClient, httpClient, blinkingFrequencyUrl);
    Serial.println("Frequency: " + blinkingFrequency);

    if (blinkingFrequency == "Error") {
      Serial.println("Error getting blinking frequency");
      return;
    }

    String messageData = getRequest(wifiClient, httpClient, messageDataUrl);
    Serial.println("Data: " + messageData);

    if (messageData == "Error") {
      Serial.println("Error getting message data");
      return;
    }

    const unsigned int dataSize = messageData.length();
    uint8_t dataBytes[dataSize];
    for (int i = 0; i < dataSize; i++) {
      dataBytes[i] = messageData[i] - '0';
    }

    sendBinary(blinkingFrequency.toFloat(), dataBytes, dataSize);
  }
}

//==============================================================================

void setupWiFi() {
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Connecting to WiFi...");

  while (WiFiClass::status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  Serial.println("WiFi connection established.");
}

void sendString(const float blinkingFrequency, const String& messageData) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending message data");

  const auto bitWaitTime = static_cast<unsigned long>(SAMPLE_RATE / blinkingFrequency);  // Time in microseconds for each bit
  unsigned long startTime = micros();

  const unsigned int dataSize = messageData.length();
  for (unsigned int i = 0; i < dataSize; i++) {
    const uint8_t byte = messageData[i];  // Character of the message data string

    for (int j = 7; j >= 0; j--) {
      digitalWrite(LIGHT_PIN, (byte >> j) & 1);  // Send the bit

      while ((micros() - startTime) < bitWaitTime) {
        // Wait for the bit time to pass
      }
      startTime += bitWaitTime;
    }
  }

  Serial.println("Data sent");
  digitalWrite(DEBUG_LED_PIN, LOW);
}

void sendBinary(const float blinkingFrequency, const uint8_t* messageData, const unsigned int dataSize) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending message data");

  const auto bitWaitTime = static_cast<unsigned long>(SAMPLE_RATE / blinkingFrequency);  // Time in microseconds for each bit
  unsigned long startTime = micros();

  for (int i = 0; i < dataSize; i++) {
    digitalWrite(LIGHT_PIN, messageData[i] ? HIGH : LOW);  // Send the bit

    while ((micros() - startTime) < bitWaitTime) {
      // Wait for the bit time to pass
    }
    startTime += bitWaitTime;
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

    return "Error";
  }

  Serial.printf("[HTTP] GET... code: %d", responseCode);

  String responsePayload = httpClient.getString();
  Serial.println("Received payload: " + responsePayload);

  return responsePayload;
}
