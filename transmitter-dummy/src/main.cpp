#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>

//==============================================================================

#define LIGHT_PIN (18) // ESP32 Pin to send light pulses
#define DEBUG_LED_PIN (19) // ESP32 Pin for debugging

//==============================================================================

const char* wifiSsid = "xxxxxx";
const char* wifiPassword = "xxxxxx";

String apiServerBaseUrl = "http://xxxxxx:5000"; // Change to server's ip address

String sendModeUrl = apiServerBaseUrl + "/get_send_mode";
String blinkingFrequencyUrl = apiServerBaseUrl + "/get_blinking_frequency";
String messageDataUrl = apiServerBaseUrl + "/get_message_data";

//==============================================================================

void send_string(float blinkingFrequency, String& messageData);
void send_binary(float blinkingFrequency, uint8_t* messageData, int dataSize);
String get_request(WiFiClient& wifiClient, HTTPClient& httpClient, String target_url);

//==============================================================================

void setup() {
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(DEBUG_LED_PIN, OUTPUT);

  Serial.begin(9600);

  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println("WiFi connection established.");
}

void loop() {
  WiFiClient wifiClient;
  HTTPClient httpClient;

  String sendMode;
  sendMode = get_request(wifiClient, httpClient, sendModeUrl);

  if (sendMode == "Error") {
    Serial.println("Error getting send mode");
    return;
  }
  else {
    Serial.println("Send mode: " + sendMode);
  }

  if (sendMode == "send_string") {
    String blinkingFrequency = get_request(wifiClient, httpClient, blinkingFrequencyUrl);
    if (blinkingFrequency == "Error") {
      Serial.println("Error getting blinking frequency");
      return;
    }
    else {
      Serial.println("Frequency: " + blinkingFrequency);
    }

    String messageData = get_request(wifiClient, httpClient, messageDataUrl);
    if (messageData == "Error") {
      Serial.println("Error getting message data");
      return;
    }
    else {
      Serial.println("Data: " + messageData);
    }

    send_string(blinkingFrequency.toFloat(), messageData);
  }

  if (sendMode == "send_binary") {
    String blinkingFrequency = get_request(wifiClient, httpClient, blinkingFrequencyUrl);
    if (blinkingFrequency == "Error") {
      Serial.println("Error getting blinking frequency");
      return;
    }
    else {
      Serial.println("Frequency: " + blinkingFrequency);
    }

    String messageData = get_request(wifiClient, httpClient, messageDataUrl);
    if (messageData == "Error") {
      Serial.println("Error getting message data");
      return;
    }
    else {
      Serial.println("Data: " + messageData);
    }

    int dataSize = messageData.length();
    uint8_t dataBytes[dataSize];
    for (int i = 0; i < dataSize; i++) {
      dataBytes[i] = messageData[i] - '0';
    }

    send_binary(blinkingFrequency.toFloat(), dataBytes, dataSize);
  }
}

//==============================================================================

void send_string(float blinkingFrequency, String& messageData) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending message data");

  unsigned long bitWaitTime = 1000000 / blinkingFrequency; // Time in microseconds for each bit
  unsigned long startTime = micros();

  int dataSize = messageData.length();
  for (int i = 0; i < dataSize; i++) {
    uint8_t byte = messageData[i]; // Character of the message data string

    for (int j = 7; j >= 0; j--) {
      digitalWrite(LIGHT_PIN, (byte >> j) & 1); // Send the bit

      while ((micros() - startTime) < bitWaitTime); // Wait for the bit time to pass
      startTime += bitWaitTime;
    }
  }

  Serial.println("Data sent");
  digitalWrite(DEBUG_LED_PIN, LOW);
}

void send_binary(float blinkingFrequency, uint8_t* messageData, int dataSize) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending message data");

  unsigned long bitWaitTime = 1000000 / blinkingFrequency; // Time in microseconds for each bit
  unsigned long startTime = micros();

  for (int i = 0; i < dataSize; i++) {
    digitalWrite(LIGHT_PIN, messageData[i] ? HIGH : LOW); // Send the bit

    while ((micros() - startTime) < bitWaitTime); // Wait for the bit time to pass
    startTime += bitWaitTime;
  }

  Serial.println("Data sent");
  digitalWrite(DEBUG_LED_PIN, LOW);
}

String get_request(WiFiClient& wifiClient, HTTPClient& httpClient, String target_url) {
  Serial.println("Sending http request");
  if (httpClient.begin(wifiClient, target_url)) {
    Serial.println("URL initialized");
  }

  int responseCode = httpClient.GET();
  if (responseCode > 0) {
    Serial.printf("[HTTP] GET... code: %d", responseCode);

    if (responseCode == HTTP_CODE_OK) {
      String responsePayload = httpClient.getString();
      Serial.println("Received payload: " + responsePayload);
      return responsePayload;
    }
    else {
      Serial.printf("ERROR: %s", httpClient.errorToString(responseCode).c_str());
      return "Error";
    }
  }
  else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", httpClient.errorToString(responseCode).c_str());
  }

  return "Error";
}
