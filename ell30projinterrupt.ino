
#include <Arduino.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/spi_master.h"

#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCLK 18
#define PIN_CS   5

#define SPI_CLOCK 1000000

#define SAMPLE_INTERVAL_US 10000  // 100 Hz

#define BATCH_SIZE 256
#define CHUNK_SIZE 32
#define NUM_CHUNKS (BATCH_SIZE / CHUNK_SIZE)

#define SPI_HOST_USED SPI2_HOST

spi_device_handle_t spiHandle;

uint8_t txBuffer[BATCH_SIZE];
uint8_t rxBuffer[BATCH_SIZE];

spi_transaction_t transactions[NUM_CHUNKS];

uint32_t totalSamples = 0;
uint32_t totalBytes = 0;
uint32_t errors = 0;
uint32_t sleepCount = 0;

int64_t nextSampleUs;
uint32_t lastReport;

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_OFF);
  btStop();

  // Prepare test data
  for (int i = 0; i < BATCH_SIZE; i++) {
    txBuffer[i] = (i * 37 + 13) & 0xFF;
  }

  // Initialize SPI bus WITHOUT DMA
  spi_bus_config_t busConfig = {};

  busConfig.mosi_io_num = PIN_MOSI;
  busConfig.miso_io_num = PIN_MISO;
  busConfig.sclk_io_num = PIN_SCLK;
  busConfig.quadwp_io_num = -1;
  busConfig.quadhd_io_num = -1;
  busConfig.max_transfer_sz = CHUNK_SIZE;

  esp_err_t result = spi_bus_initialize(
    SPI_HOST_USED,
    &busConfig,
    SPI_DMA_DISABLED
  );

  if (result != ESP_OK) {
    Serial.printf("SPI init failed: %d\n", result);
    while (true) delay(1000);
  }

  // Configure SPI device
  spi_device_interface_config_t deviceConfig = {};

  deviceConfig.clock_speed_hz = SPI_CLOCK;
  deviceConfig.mode = 0;
  deviceConfig.spics_io_num = PIN_CS;
  deviceConfig.queue_size = NUM_CHUNKS;

  result = spi_bus_add_device(
    SPI_HOST_USED,
    &deviceConfig,
    &spiHandle
  );

  if (result != ESP_OK) {
    Serial.printf("Device init failed: %d\n", result);
    while (true) delay(1000);
  }

  // Prepare eight transactions
  for (int i = 0; i < NUM_CHUNKS; i++) {
    memset(&transactions[i], 0, sizeof(spi_transaction_t));

    transactions[i].length = CHUNK_SIZE * 8;
    transactions[i].tx_buffer = &txBuffer[i * CHUNK_SIZE];
    transactions[i].rx_buffer = &rxBuffer[i * CHUNK_SIZE];
  }

  Serial.println("\nSPI INTERRUPT + LIGHT SLEEP TEST");
  Serial.println("DMA: DISABLED");
  Serial.println("Sampling frequency: 100 Hz");
  Serial.println("SPI clock: 1 MHz");
  Serial.println("Batch size: 256 bytes");
  Serial.println("Light sleep: ENABLED");

  lastReport = millis();
  nextSampleUs = esp_timer_get_time() + SAMPLE_INTERVAL_US;
}

void loop() {

  // Sleep until the next sample deadline
  while (true) {

    int64_t now = esp_timer_get_time();

    if (now >= nextSampleUs) {
      break;
    }

    uint64_t remainingUs = nextSampleUs - now;

    esp_sleep_enable_timer_wakeup(remainingUs);

    esp_err_t sleepResult = esp_light_sleep_start();

    if (sleepResult == ESP_OK) {
      sleepCount++;
    } else {
      errors++;
      break;
    }
  }

  // Queue eight interrupt-driven SPI transactions
  bool transferOK = true;

  for (int i = 0; i < NUM_CHUNKS; i++) {

    esp_err_t result = spi_device_queue_trans(
      spiHandle,
      &transactions[i],
      portMAX_DELAY
    );

    if (result != ESP_OK) {
      transferOK = false;
      break;
    }
  }

  // Collect completed transactions
  if (transferOK) {

    for (int i = 0; i < NUM_CHUNKS; i++) {

      spi_transaction_t *completedTransaction;

      esp_err_t result = spi_device_get_trans_result(
        spiHandle,
        &completedTransaction,
        portMAX_DELAY
      );

      if (result != ESP_OK) {
        transferOK = false;
        break;
      }
    }
  }

  // Verify loopback
  if (transferOK) {

    for (int i = 0; i < BATCH_SIZE; i++) {

      if (rxBuffer[i] != txBuffer[i]) {
        errors++;
        break;
      }
    }

  } else {
    errors++;
  }

  totalSamples++;
  totalBytes += BATCH_SIZE;

  // Maintain 100 Hz schedule
  nextSampleUs += SAMPLE_INTERVAL_US;

  // Skip missed deadlines rather than catching up in a burst
  int64_t now = esp_timer_get_time();

  if (nextSampleUs <= now) {
    nextSampleUs = now + SAMPLE_INTERVAL_US;
  }

  // Report every 5 seconds
  if (millis() - lastReport >= 5000) {

    float elapsed = (millis() - lastReport) / 1000.0;

    Serial.printf(
      "Interrupt+Sleep | Samples/sec: %.2f | Throughput: %.2f KB/s | Errors: %lu | Sleep cycles: %lu\n",
      totalSamples / elapsed,
      (totalBytes / 1024.0) / elapsed,
      (unsigned long)errors,
      (unsigned long)sleepCount
    );

    totalSamples = 0;
    totalBytes = 0;
    errors = 0;
    sleepCount = 0;

    lastReport = millis();
  }
}
