#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <atomic>

#include "driver/spi_master.h"

//==============================================================================

#define SPI_MOSI (23)
#define SPI_MISO (19)
#define SPI_SCK (18)
#define SPI_CS (5)

#define SIGNAL_PIN (5)

#define SAMPLE_RATE (1000000)  // 1 MHz
#define BUFFER_SIZE (1024)
#define CIRCULAR_BUFFER_SIZE (4096)  // Tamaño total del buffer circular

//==============================================================================

const String wifiSsid = "xxxxxx";
const String wifiPassword = "xxxxxx";
const auto * const serverHostname = "http://xxxxxx:5001";  // Raspberry Pi IP
constexpr int serverPort = 5001;
const String binaryEndpoint = serverHostname + String("/receive_binary");

WiFiClient wifiClient;
spi_device_handle_t spiDeviceHandle;
std::array<uint16_t, CIRCULAR_BUFFER_SIZE>circularBuffer;
std::atomic_ushort packageCounter(0);
volatile size_t bufferHeadIndex = 0;
volatile size_t bufferTailIndex = 0;
volatile bool bufferIsReady = false;
unsigned long lastReconnectionAttempt = 0;
TaskHandle_t sendBufferHandler = NULL;

//==============================================================================

uint16_t calculateChecksum(const uint16_t *pData, size_t length);
void IRAM_ATTR spiTransmissionCompletedCallback(spi_transaction_t *trans);
void setupSPI();
void readADC();
void setupWiFi();
void reconnectWiFi();
void sendBuffer(void *);
String postRequest(WiFiClient& wifiClient, HTTPClient& httpClient, const String& targetUrl, uint8_t* payload,
                   size_t size);
void readPhotorresistor();
//==============================================================================

void setup() {
  Serial.begin(9600);

  pinMode(SIGNAL_PIN, INPUT);
  Serial.println("Starting wifi");
  // setupSPI();
  setupWiFi();
  xTaskCreatePinnedToCore(sendBuffer, "sendBuffer", 2048, nullptr, 1, &sendBufferHandler, 0);
}

void loop() { // Loop uses core 1
  //reconnectWiFi();
  //readADC();
  readPhotorresistor();
}

//==============================================================================

uint16_t calculateChecksum(const uint16_t *pData, const size_t length) {
  uint16_t checksum = 0;

  for (size_t i = 0; i < length; i++) {
    checksum ^= pData[i];
  }

  return checksum;
}

void IRAM_ATTR spiTransmissionCompletedCallback(spi_transaction_t *trans) {
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

  spi_bus_initialize(HSPI_HOST, &spiBusConfig, SPI_DMA_CH_AUTO);  // Activar canal DMA
  spi_bus_add_device(HSPI_HOST, &spiDeviceInterfaceConfig, &spiDeviceHandle);
}

void readADC() {
  spi_transaction_t spiTransaction = {};

  memset(&spiTransaction, 0, sizeof(spiTransaction));
  spiTransaction.length = BUFFER_SIZE * 16;  // 16 bits por muestra
  spiTransaction.rx_buffer = &circularBuffer[bufferHeadIndex];
  spiTransaction.flags = SPI_TRANS_USE_RXDATA;  // Asegurar uso de DMA
  spi_device_queue_trans(spiDeviceHandle, &spiTransaction, portMAX_DELAY);

  bufferHeadIndex = (bufferHeadIndex + BUFFER_SIZE) % CIRCULAR_BUFFER_SIZE;
  sendBuffer(nullptr);
}

void setupWiFi() {
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Connecting to WiFi...");

  while (WiFiClass::status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  Serial.println("WiFi connection established.");
  //wifiClient.connect(serverHostname, serverPort);
}

void reconnectWiFi() {
  if (
      (WiFiClass::status() == WL_CONNECTED)
      || (millis() - lastReconnectionAttempt <= 5000)
  ) {
    return;
  }

  Serial.println("WiFi connection lost, trying to reconnect...");
  lastReconnectionAttempt = millis();
  WiFi.disconnect();
  setupWiFi();
}

void sendBuffer(void *params) {
  while (true) {
    delay(1);
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

  if (httpClient.begin(wifiClient, targetUrl)) {
    Serial.println("URL initialized");
  }

  const int responseCode = httpClient.POST(payload, size);

  if (responseCode != HTTP_CODE_OK) {
    Serial.printf(
        "[HTTP] POST... failed, error: %s\n",
        HTTPClient::errorToString(responseCode).c_str()
    );

    return "Error";
  }

  return String(responseCode);
}

void readPhotorresistor() {
  while (true) {
    uint16_t currentValue = 0; // Next value to insert in the buffer.
    for (int i = 15; i >= 0; --i) {
      const int readBit = digitalRead(SIGNAL_PIN) ? HIGH : LOW;
      currentValue |= readBit << i;
      Serial.print(readBit ? 1 : 0);
      // Wait time between reads
      delayMicroseconds(1);
    }
    //Serial.print(currentValue);
    circularBuffer[bufferHeadIndex++] = currentValue;
    if (bufferHeadIndex % BUFFER_SIZE == 0) {
      Serial.println( " Finished package");
      packageCounter++;
      if (bufferHeadIndex == CIRCULAR_BUFFER_SIZE) {
        bufferHeadIndex = 0;
      }
    }
  }
}