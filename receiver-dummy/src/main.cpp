#include <SPI.h>
#include <WiFi.h>

#include "driver/spi_master.h"

//==============================================================================

#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18
#define SPI_CS 5

#define SAMPLE_RATE 1000000  // 1 MHz
#define BUFFER_SIZE 1024
#define CIRCULAR_BUFFER_SIZE 4096  // Tamaño total del buffer circular

//==============================================================================

const char* ssid = "ESP32_NETWORK";
const char* password = "password1234";
const char* serverIP = "192.168.1.100";  // Raspberry Pi IP
const int serverPort = 12345;

WiFiClient client;
spi_device_handle_t spi;
uint16_t circular_buffer[CIRCULAR_BUFFER_SIZE];
volatile size_t head = 0;
volatile size_t tail = 0;
volatile bool buffer_ready = false;
unsigned long lastReconnectAttempt = 0;

//==============================================================================

uint16_t calculateChecksum(uint16_t* data, size_t length);
void IRAM_ATTR spi_callback(spi_transaction_t *trans);
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
  if (buffer_ready) {
    buffer_ready = false;
  }
}

//==============================================================================

uint16_t calculateChecksum(uint16_t* data, size_t length) {
  uint16_t checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void IRAM_ATTR spi_callback(spi_transaction_t *trans) {
  buffer_ready = true;
}

void setupSPI() {
  spi_bus_config_t buscfg = {
      .mosi_io_num = SPI_MOSI,
      .miso_io_num = SPI_MISO,
      .sclk_io_num = SPI_SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = BUFFER_SIZE * sizeof(uint16_t)};

  spi_device_interface_config_t devcfg = {
      .mode = 0,
      .clock_speed_hz = SAMPLE_RATE * 16,  // 16 MHz SPI Clock
      .spics_io_num = SPI_CS,
      .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX,
      .queue_size = 1,
      .pre_cb = NULL,
      .post_cb = spi_callback};

  spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);  // Activar canal DMA
  spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
}

void readADC() {
  spi_transaction_t trans = {};
  memset(&trans, 0, sizeof(trans));
  trans.length = BUFFER_SIZE * 16;  // 16 bits por muestra
  trans.rx_buffer = &circular_buffer[head];
  trans.flags = SPI_TRANS_USE_RXDATA;  // Asegurar uso de DMA
  spi_device_queue_trans(spi, &trans, portMAX_DELAY);
  head = (head + BUFFER_SIZE) % CIRCULAR_BUFFER_SIZE;
  sendBuffer();
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  client.connect(serverIP, serverPort);
}

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED && (millis() - lastReconnectAttempt > 5000)) {
    Serial.println("Reconnecting to WiFi...");
    lastReconnectAttempt = millis();
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    client.connect(serverIP, serverPort);
  }
}

void sendBuffer() {
  while (tail != head) {
    uint16_t checksum = calculateChecksum(&circular_buffer[tail], BUFFER_SIZE);
    if (client.connected()) {
      client.write((uint8_t*)&circular_buffer[tail], BUFFER_SIZE * sizeof(uint16_t));
      client.write((uint8_t*)&checksum, sizeof(checksum));
      client.flush();  // Asegura que los datos se envíen inmediatamente
    }
    tail = (tail + BUFFER_SIZE) % CIRCULAR_BUFFER_SIZE;
  }
}
