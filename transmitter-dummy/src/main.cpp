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

const String messageDataUrl = apiServerBaseUrl + "/get_message_data";
const String blinkingFrequencyUrl = apiServerBaseUrl + "/get_blinking_frequency";

//==============================================================================

void sendMessage(const String &messageData, const float &blinkingFrequency);
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

  const String messageData = getRequest(wifiClient, httpClient, messageDataUrl);
  Serial.println("Message data: " + messageData);

  if (messageData == "Error") {
    Serial.println("Error getting message data");
    return;
  }

  const String blinkingFrequency = getRequest(wifiClient, httpClient, blinkingFrequencyUrl);
  Serial.println("Blinking frequency: " + blinkingFrequency);

  if (blinkingFrequency == "Error") {
    Serial.println("Error getting blinking frequency");
    return;
  }

  sendMessage(messageData, blinkingFrequency.toFloat());
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

void sendMessage(const String &messageData, const float &blinkingFrequency) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending message data");

  const auto bitWaitTime = static_cast<unsigned long>(SAMPLE_RATE / blinkingFrequency);  // Time in microseconds for each bit
  unsigned long startTime = micros();

  for (const auto messageBit : messageData) {
    digitalWrite(LIGHT_PIN, messageBit ? HIGH : LOW);  // Send the bit

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
