#include <SPI.h>
#include <WiFi.h>

#include "driver/spi_master.h"

//==============================================================================

#define SPI_MOSI (23)
#define SPI_MISO (19)
#define SPI_SCK (18)
#define SPI_CS (5)

#define SAMPLE_RATE (1000000)  // 1 MHz
#define BUFFER_SIZE (1024)
#define CIRCULAR_BUFFER_SIZE (4096)  // Tamaño total del buffer circular

//==============================================================================

const String wifiSsid = "xxxxxx";
const String wifiPassword = "xxxxxx";
const auto * const serverHostname = "xxxxxx";  // Raspberry Pi IP
constexpr int serverPort = 12345;

WiFiClient wifiClient;
spi_device_handle_t spiDeviceHandle;
std::array<uint16_t, CIRCULAR_BUFFER_SIZE>circularBuffer;
volatile size_t bufferHeadIndex = 0;
volatile size_t bufferTailIndex = 0;
volatile bool bufferIsReady = false;
unsigned long lastReconnectionAttempt = 0;

//==============================================================================

uint16_t calculateChecksum(const uint16_t *pData, size_t length);
void IRAM_ATTR spiTransmissionCompletedCallback(spi_transaction_t *trans);
void setupSPI();
void readADC();
void setupWiFi();
void reconnectWiFi();
void sendBuffer();

//==============================================================================

void setup() {
  Serial.begin(115200);
  setupSPI();
  setupWiFi();
}

void loop() {
  reconnectWiFi();
  readADC();

  if (bufferIsReady) {
    bufferIsReady = false;
  }
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
  sendBuffer();
}

void setupWiFi() {
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Connecting to WiFi...");

  while (WiFiClass::status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  Serial.println("WiFi connection established.");
  wifiClient.connect(serverHostname, serverPort);
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

void sendBuffer() {
  while (bufferTailIndex != bufferHeadIndex) {
    uint16_t checksum = calculateChecksum(
        &circularBuffer[bufferTailIndex],
        BUFFER_SIZE
    );

    if (wifiClient.connected()) {
      wifiClient.write(
          reinterpret_cast<uint8_t *>(&circularBuffer[bufferTailIndex]),
          BUFFER_SIZE * sizeof(uint16_t)
      );
      wifiClient.write(
          reinterpret_cast<uint8_t *>(&checksum),
          sizeof(checksum)
      );
      wifiClient.flush();  // Asegura que los datos se envíen inmediatamente
    }

    bufferTailIndex = (bufferTailIndex + BUFFER_SIZE) % CIRCULAR_BUFFER_SIZE;
  }
}
