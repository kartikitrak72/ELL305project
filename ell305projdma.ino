
#include <Arduino.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/spi_master.h"

#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCLK 18
#define PIN_CS   5

#define SPI_CLOCK 1000000

#define SAMPLE_INTERVAL_US 10000  // 100 Hz
#define BUFFER_SIZE 256

#define SPI_HOST_USED SPI2_HOST

spi_device_handle_t spiHandle;

DMA_ATTR uint8_t txBuffer[BUFFER_SIZE] __attribute__((aligned(4)));
DMA_ATTR uint8_t rxBuffer[BUFFER_SIZE] __attribute__((aligned(4)));

spi_transaction_t transaction;

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

  // Prepare repeatable test data
  for (int i = 0; i < BUFFER_SIZE; i++) {
    txBuffer[i] = (i * 37 + 13) & 0xFF;
  }

  memset(rxBuffer, 0, BUFFER_SIZE);
  memset(&transaction, 0, sizeof(transaction));

  // Initialize SPI bus with DMA
  spi_bus_config_t busConfig = {};

  busConfig.mosi_io_num = PIN_MOSI;
  busConfig.miso_io_num = PIN_MISO;
  busConfig.sclk_io_num = PIN_SCLK;
  busConfig.quadwp_io_num = -1;
  busConfig.quadhd_io_num = -1;
  busConfig.max_transfer_sz = BUFFER_SIZE;

  esp_err_t result = spi_bus_initialize(
    SPI_HOST_USED,
    &busConfig,
    SPI_DMA_CH_AUTO
  );

  if (result != ESP_OK) {
    Serial.printf("SPI initialization failed: %d\n", result);
    while (true) delay(1000);
  }

  // Configure SPI device
  spi_device_interface_config_t deviceConfig = {};

  deviceConfig.clock_speed_hz = SPI_CLOCK;
  deviceConfig.mode = 0;
  deviceConfig.spics_io_num = PIN_CS;
  deviceConfig.queue_size = 1;

  result = spi_bus_add_device(
    SPI_HOST_USED,
    &deviceConfig,
    &spiHandle
  );

  if (result != ESP_OK) {
    Serial.printf("Device initialization failed: %d\n", result);
    while (true) delay(1000);
  }

  // Configure DMA transaction
  transaction.length = BUFFER_SIZE * 8;
  transaction.tx_buffer = txBuffer;
  transaction.rx_buffer = rxBuffer;

  Serial.println();
  Serial.println("ESP32 SPI DMA + LIGHT SLEEP TEST");
  Serial.println("DMA: ENABLED");
  Serial.println("Sampling frequency: 100 Hz");
  Serial.println("SPI clock: 1 MHz");
  Serial.println("Batch size: 256 bytes");
  Serial.println("Light sleep: ENABLED");

  lastReport = millis();
  nextSampleUs = esp_timer_get_time() + SAMPLE_INTERVAL_US;
}

void loop() {

  // Sleep until the next 10 ms sample deadline
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

  // Start DMA transfer
  esp_err_t result = spi_device_queue_trans(
    spiHandle,
    &transaction,
    portMAX_DELAY
  );

  bool transferOK = (result == ESP_OK);

  // Wait for DMA completion
  if (transferOK) {

    spi_transaction_t *completedTransaction = nullptr;

    result = spi_device_get_trans_result(
      spiHandle,
      &completedTransaction,
      portMAX_DELAY
    );

    if (result != ESP_OK ||
        completedTransaction != &transaction) {
      transferOK = false;
    }
  }

  // Verify loopback
  if (transferOK) {

    for (int i = 0; i < BUFFER_SIZE; i++) {
      if (rxBuffer[i] != txBuffer[i]) {
        transferOK = false;
        break;
      }
    }
  }

  if (!transferOK) {
    errors++;
  }

  totalSamples++;
  totalBytes += BUFFER_SIZE;

  // Advance sampling schedule
  nextSampleUs += SAMPLE_INTERVAL_US;

  // Avoid burst catch-up if a deadline was missed
  int64_t now = esp_timer_get_time();

  if (nextSampleUs <= now) {
    nextSampleUs = now + SAMPLE_INTERVAL_US;
  }

  // Report every 5 seconds
  if (millis() - lastReport >= 5000) {

    float elapsed = (millis() - lastReport) / 1000.0;

    Serial.printf(
      "DMA+Sleep | Samples/sec: %.2f | Throughput: %.2f KB/s | Errors: %lu | Sleep cycles: %lu\n",
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
