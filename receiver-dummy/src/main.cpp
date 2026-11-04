/**
 * TEIDESAT Dummies - Receiver Firmware
 * Runs on an ESP32 WROOM 32
 * - Core 1 dedicates entirely to high-speed digital reading at 1MHz
 * - Core 0 asynchronously packages and sends the data over Wi-Fi via HTTP POST
 */

#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <atomic>

#include "driver/spi_master.h"
#include "driver/rmt_rx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"

#include "secrets.h"

//==============================================================================

#define SPI_MOSI (23)
#define SPI_MISO (19)
#define SPI_SCK (18)
#define SPI_CS (5)

#define SIGNAL_PIN (5) // Photodiode input

#define SAMPLE_RATE (100000)        // 1 MHz target sampling rate
#define BUFFER_SIZE (1024)           // Samples per HTTP payload
#define CIRCULAR_BUFFER_SIZE (4096)  // Total size of the ring buffer (4 payloads)

#define RMT_MEM_BLOCK_SYMBOLS 128

IPAddress localIP(10, 42, 100, 10);  // Local IP address
IPAddress gatewayIP(10, 42, 0, 2);   // Gateway IP address
IPAddress subnetMask(255, 255, 0, 0); // Subnet mask

//==============================================================================

// Defined at 'secrets.h' file
const String wifiSsid = WIFI_SSID;
const String wifiPassword = WIFI_PASSWORD;
const String apiServerBaseUrl = API_SERVER_BASE_URL;

// Endpoint at receiver server
const String binaryEndpoint = apiServerBaseUrl + "/receive_binary";

WiFiClient wifiClient;
spi_device_handle_t spiDeviceHandle;

std::array<uint16_t, CIRCULAR_BUFFER_SIZE>circularBuffer;
std::atomic_ushort packageCounter(0);
volatile size_t bufferHeadIndex = 0;
volatile size_t bufferTailIndex = 0;
volatile bool bufferIsReady = false;
unsigned long lastReconnectionAttempt = 0;
TaskHandle_t sendBufferHandler = NULL;

constexpr unsigned long kReconnectIntervalMs = 5000;
constexpr int kWiFiConnectTimeoutMs = 15000;

// RMT related
static rmt_channel_handle_t rx_chan = nullptr;
static rmt_symbol_word_t rx_buffer[RMT_MEM_BLOCK_SYMBOLS];
static size_t received_symbols_len = 0;
rmt_receive_config_t ook_config_2khz = { // TODO: Adaptative in the future
  .signal_range_min_ns = 50000,
  .signal_range_max_ns = 8000000,
};
SemaphoreHandle_t rx_semaphore = nullptr;
#define RMT_TICK_FREQ_HZ 1000000UL
#define SIGNAL_FREQ_HZ 2000UL

constexpr uint32_t TICKS_PER_BIT = (RMT_TICK_FREQ_HZ / SIGNAL_FREQ_HZ);

//==============================================================================

uint16_t calculateChecksum(const uint16_t *pData, size_t length);
void IRAM_ATTR spiTransmissionCompletedCallback(spi_transaction_t *trans);
void setupSPI();
void readADC();
bool setupWiFi();
bool reconnectWiFi();
bool connectWiFiWithTimeout();
void sendBuffer(void *);
String postRequest(WiFiClient& wifiClient, HTTPClient& httpClient, const String& targetUrl, uint8_t* payload,
                   size_t size);
void readPhotorresistor();
void readPhotodiode();
bool ensureWiFiConnected();
void setupRMT();
bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data);

//==============================================================================

void setup() {
  Serial.begin(115200);

  pinMode(SIGNAL_PIN, INPUT);
  Serial.println("Starting wifi");
  // setupSPI();
  setupWiFi();
  setupRMT();

  xTaskCreatePinnedToCore(sendBuffer, "sendBuffer", 2048, nullptr, 1, &sendBufferHandler, 0);
}

// Core 1 loop by default
void loop() {
  reconnectWiFi();
  //readADC();
  //readPhotorresistor();
  readPhotodiode();
}

//==============================================================================

// Calculate a simple XOR checksum for the given data
uint16_t calculateChecksum(const uint16_t *pData, const size_t length) {
  uint16_t checksum = 0;

  for (size_t i = 0; i < length; i++) {
    checksum ^= pData[i];
  }

  return checksum;
}

void spiTransmissionCompletedCallback(spi_transaction_t *trans) {
  bufferIsReady = true;
}

void setupSPI() {
  constexpr spi_bus_config_t spiBusConfig = {
      .mosi_io_num = SPI_MOSI,
      .miso_io_num = SPI_MISO,
      .sclk_io_num = SPI_SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = BUFFER_SIZE * sizeof(uint16_t)
  };

  constexpr spi_device_interface_config_t spiDeviceInterfaceConfig = {
      .mode = 0,
      .clock_speed_hz = SAMPLE_RATE * 16,  // 16 MHz SPI Clock
      .spics_io_num = SPI_CS,
      .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
      .queue_size = 1,
      .pre_cb = nullptr,
      .post_cb = spiTransmissionCompletedCallback
  };

  spi_bus_initialize(HSPI_HOST, &spiBusConfig, SPI_DMA_CH_AUTO);
  spi_bus_add_device(HSPI_HOST, &spiDeviceInterfaceConfig, &spiDeviceHandle);
}

void readADC() {
  spi_transaction_t spiTransaction = {};

  memset(&spiTransaction, 0, sizeof(spiTransaction));
  spiTransaction.length = BUFFER_SIZE * 16;  // 16 bits per sample
  spiTransaction.rx_buffer = &circularBuffer[bufferHeadIndex];
  spiTransaction.flags = SPI_TRANS_USE_RXDATA;  // Ensure DMA usage
  spi_device_queue_trans(spiDeviceHandle, &spiTransaction, portMAX_DELAY);

  bufferHeadIndex = (bufferHeadIndex + BUFFER_SIZE) % CIRCULAR_BUFFER_SIZE;
  sendBuffer(nullptr);
}

bool setupWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
  WiFi.disconnect(true, true);
  delay(250);

  if (!connectWiFiWithTimeout()) {
    Serial.println("WiFi connect timeout.");
    WiFi.disconnect(true, true);
    //WiFi.mode(WIFI_OFF);
    //delay(500);
    return false;
  }
  return true;
}

bool connectWiFiWithTimeout() {
  if(!WiFi.config(localIP, gatewayIP, subnetMask)) {
    Serial.println("STA Failed to configure");
    return false;
  }

  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Connecting to WiFi...");

  const unsigned long connectStart = millis();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);

    if (millis() - connectStart >= kWiFiConnectTimeoutMs) {
      Serial.println(" timeout");
      return false;
    }
  }

  Serial.println("WiFi connection established.");
  return true;
}

bool ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  return reconnectWiFi();
}

bool reconnectWiFi() {
  if (
      (WiFi.status() == WL_CONNECTED)
      || (millis() - lastReconnectionAttempt <= kReconnectIntervalMs)
  ) {
    return true;
  }

  Serial.println("WiFi connection lost, trying to reconnect...");
  lastReconnectionAttempt = millis();
  WiFi.disconnect(true, true);
  return setupWiFi();
}

// Core 0
void sendBuffer(void *params) {
  while (true) {
    delay(1);
    // Process all pending packages if Core 1 has filled them
    while (packageCounter != 0) {
      uint16_t checksum = calculateChecksum(
          &circularBuffer[bufferTailIndex],
          BUFFER_SIZE
      );

      //if (wifiClient.connected()) {
        /*
        wifiClient.write(
            reinterpret_cast<uint8_t *>(&circularBuffer[bufferTailIndex]),
            BUFFER_SIZE * sizeof(uint16_t)
        );
        wifiClient.write(
            reinterpret_cast<uint8_t *>(&checksum),
            sizeof(checksum)
        );*/
        HTTPClient httpClient;

        postRequest(wifiClient, httpClient, binaryEndpoint,
                    reinterpret_cast<uint8_t *>(&circularBuffer[bufferTailIndex]), BUFFER_SIZE * sizeof(uint16_t));
        //wifiClient.flush();  // Asegura que los datos se envíen inmediatamente
      //}

      bufferTailIndex = (bufferTailIndex + BUFFER_SIZE) % CIRCULAR_BUFFER_SIZE;
      packageCounter--;
    }
  }
}

String postRequest(WiFiClient& wifiClient, HTTPClient& httpClient, const String& targetUrl, uint8_t* payload,
                   size_t size) {
  Serial.println("Sending http request");

  if (!ensureWiFiConnected()) {
    Serial.println("Error: Post not send. WiFi Disconnect");
    return "Error";
  }

  httpClient.setReuse(false);
  //httpClient.addHeader("Connection", "close");

  if (!httpClient.begin(wifiClient, targetUrl)) {
    Serial.println("Failed to initialize URL");
    wifiClient.stop();
    return "Error";
  }

  Serial.println("URL initialized");

  const int responseCode = httpClient.POST(payload, size);

  if (responseCode != HTTP_CODE_OK) {
    Serial.printf(
        "[HTTP] POST... failed, error: %s\n",
        HTTPClient::errorToString(responseCode).c_str()
    );

    httpClient.end();
    wifiClient.stop();

    return "Error";
  }

  httpClient.end();
  //wifiClient.stop();
  return String(responseCode);
}

void readPhotorresistor() {
  while (true) {
    uint16_t currentValue = 0; // Next value to insert in the buffer.
    for (int i = 15; i >= 0; --i) {
      const int readBit = digitalRead(SIGNAL_PIN) ? HIGH : LOW;
      currentValue |= readBit << i;
      //Serial.print(readBit ? 1 : 0);
      // Wait time between reads
      delayMicroseconds(1);
    }
    //Serial.print(currentValue,BIN);
    circularBuffer[bufferHeadIndex] = currentValue;
    bufferHeadIndex += 1;
    if (bufferHeadIndex % BUFFER_SIZE == 0) {
      //Serial.println( " Finished package");
      packageCounter++;
      if (bufferHeadIndex == CIRCULAR_BUFFER_SIZE) {
        bufferHeadIndex = 0;
      }
    }
  }
}

void setupRMT() {
  rx_semaphore = xSemaphoreCreateBinary();

  rmt_rx_channel_config_t rx_chan_config = {};
  rx_chan_config.gpio_num = (gpio_num_t)SIGNAL_PIN; // Data pin
  rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT; // Default clock (80 MHz)
  rx_chan_config.resolution_hz = RMT_TICK_FREQ_HZ;
  rx_chan_config.mem_block_symbols = RMT_MEM_BLOCK_SYMBOLS;  
  rx_chan_config.flags.invert_in = false; 
  rx_chan_config.flags.with_dma = false; 

  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

  // Register callback for processing
  rmt_rx_event_callbacks_t cbk = {
    .on_recv_done = rmt_rx_done_callback,
  };

  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbk, NULL));
  ESP_ERROR_CHECK(rmt_enable(rx_chan));
  ESP_ERROR_CHECK(rmt_receive(rx_chan, rx_buffer, sizeof(rx_buffer), &ook_config_2khz));
}

bool rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data)
{
  BaseType_t high_task_wakeup = pdFALSE;
  
  received_symbols_len = edata->num_symbols;

  xSemaphoreGiveFromISR(rx_semaphore, &high_task_wakeup);

  return high_task_wakeup == pdTRUE;
}

void readPhotodiode() {
  if (xSemaphoreTake(rx_semaphore, portMAX_DELAY) == pdTRUE) { 
    static uint16_t currentWord = 0;
    static uint8_t bitPosition = 0;

    for (size_t i = 0; i < received_symbols_len; i++) {
      // First level data
      uint32_t ticks0 = rx_buffer[i].duration0;
      uint8_t level0  = rx_buffer[i].level0;

      // 50% Tolerance reception
      uint32_t numBits0 = (ticks0 + (TICKS_PER_BIT / 2)) / TICKS_PER_BIT;

      for (uint32_t b = 0; b < numBits0; b++) {
        if (level0 == 1) {
          currentWord |= (1 << (15 - bitPosition));
        } // TODO: Check if currentWord |= (level0 << (15 - bitPosition)); is faster
        bitPosition++;

        // Send word when full
        if (bitPosition == 16) {
          circularBuffer[bufferHeadIndex] = currentWord;
          bufferHeadIndex = (bufferHeadIndex + 1) % CIRCULAR_BUFFER_SIZE;
          if (bufferHeadIndex % BUFFER_SIZE == 0) {
            packageCounter++;
          }
          currentWord = 0;
          bitPosition = 0;
        }
      }

      // Second level data
      uint32_t ticks1 = rx_buffer[i].duration1;
      uint8_t level1  = rx_buffer[i].level1;

      uint32_t numBits1 = (ticks1 + (TICKS_PER_BIT / 2)) / TICKS_PER_BIT;

      for (uint32_t b = 0; b < numBits1; b++) {
        if (level1 == 1) {
          currentWord |= (1 << (15 - bitPosition));
        }
        bitPosition++;

        if (bitPosition == 16) {
          circularBuffer[bufferHeadIndex] = currentWord;
          bufferHeadIndex = (bufferHeadIndex + 1) % CIRCULAR_BUFFER_SIZE;
          if (bufferHeadIndex % BUFFER_SIZE == 0) {
            packageCounter++;
          }
          currentWord = 0;
          bitPosition = 0;
        }
      }
    }

    // if the last bits don't make a full word
    if (bitPosition > 0) {
      circularBuffer[bufferHeadIndex] = currentWord;
      bufferHeadIndex = (bufferHeadIndex + 1) % CIRCULAR_BUFFER_SIZE;
      
      packageCounter++;

      currentWord = 0;
      bitPosition = 0;
    }

    ESP_ERROR_CHECK(rmt_receive(rx_chan, rx_buffer, sizeof(rx_buffer), &ook_config_2khz));
  }
}
