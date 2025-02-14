#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>

//==============================================================================

#define LIGHT_PIN (15) // ESP32 Pin to send light pulses
#define DEBUG_LED_PIN (16) // ESP32 Pin of small LED for debugging

//==============================================================================

const char* ssid = "xxxxxx";
const char* password = "xxxxxx";

String base_url = "http://transmitter-dummy:5000";

String url_send = base_url + "/send";
String url_frequency = base_url + "/frequency";
String url_data = base_url + "/data";

//==============================================================================

void send_string(float frequency, String& data);
void send_binary(float frequency, uint8_t* data, int data_size);
String get_request(WiFiClient& client, HTTPClient& http, String URL);

//==============================================================================

void setup() {
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(DEBUG_LED_PIN, OUTPUT);

  Serial.begin(9600);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println("WiFi connection established.");
}

void loop() {
  WiFiClient client;
  HTTPClient http;

  String payload;
  payload = get_request(client, http, url_send);

  if (payload == "Error") {
    Serial.println("Error getting payload");
    return;
  }

  if (payload == "send_string") {
    String frequency = get_request(client, http, url_frequency);
    if (frequency == "Error") {
      Serial.println("Error getting frequency");
      return;
    }

    String data = get_request(client, http, url_data);
    if (data == "Error") {
      Serial.println("Error getting data");
      return;
    }

    send_string(frequency.toFloat(), data);
  }

  if (payload == "send_binary") {
    String frequency = get_request(client, http, url_frequency);
    if (frequency == "Error") {
      Serial.println("Error getting frequency");
      return;
    }

    String data = get_request(client, http, url_data);
    if (data == "Error") {
      Serial.println("Error getting data");
      return;
    }

    int data_size = data.length();
    uint8_t data_bytes[data_size];
    for (int i = 0; i < data_size; i++) {
      data_bytes[i] = data[i] - '0';
    }

    send_binary(frequency.toFloat(), data_bytes, data_size);
  }
}

//==============================================================================

void send_string(float frequency, String& data) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending data");

  unsigned long bit_time = 1000000 / frequency; // Time in microseconds for each bit
  unsigned long start_time = micros();

  int data_size = data.length();
  for (int i = 0; i < data_size; i++) {
    uint8_t byte = data[i]; // Character of the data string

    for (int j = 7; j >= 0; j--) {
      digitalWrite(LIGHT_PIN, (byte >> j) & 1); // Send the bit

      while (micros() - start_time < bit_time); // Wait for the bit time to pass
      start_time += bit_time;
    }
  }

  Serial.println("Data sent");
  digitalWrite(DEBUG_LED_PIN, LOW);
}

void send_binary(float frequency, uint8_t* data, int data_size) {
  digitalWrite(DEBUG_LED_PIN, HIGH);
  Serial.println("Sending data");

  unsigned long bit_time = 1000000 / frequency; // Time in microseconds for each bit
  unsigned long start_time = micros();

  for (int i = 0; i < data_size; i++) {
    digitalWrite(LIGHT_PIN, data[i] ? HIGH : LOW); // Send the bit

    while (micros() - start_time < bit_time); // Wait for the bit time to pass
    start_time += bit_time;
  }

  Serial.println("Data sent");
  digitalWrite(DEBUG_LED_PIN, LOW);
}

String get_request(WiFiClient& client, HTTPClient& http, String URL) {
  Serial.println("Sending http request");
  if (http.begin(client, URL)) {
    Serial.println("URL initialized");
  }

  int httpCode = http.GET();
  if (httpCode > 0) {
    Serial.printf("[HTTP] GET ... code: %d", httpCode);

    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println("Reveived payload: " + payload);
      return payload;
    } else {
      Serial.printf("ERROR: %s", http.errorToString(httpCode).c_str());
      return "Error";
    }
  }

  return "Error";
}
