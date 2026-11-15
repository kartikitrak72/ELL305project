
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "esp_bt.h"

#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCLK 18
#define PIN_CS   5

#define SPI_CLOCK 1000000
#define BUFFER_SIZE 256

#define SAMPLE_INTERVAL_US 10000  // 100 Hz

SPIClass spi(VSPI);

uint8_t txBuffer[BUFFER_SIZE];
uint8_t rxBuffer[BUFFER_SIZE];

uint32_t totalBytes = 0;
uint32_t totalSamples = 0;
uint32_t errors = 0;

uint32_t lastSample;
uint32_t lastReport;

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_OFF);
  btStop();

  // Generate repeatable test data
  for (int i = 0; i < BUFFER_SIZE; i++) {
    txBuffer[i] = (i * 37 + 13) & 0xFF;
  }

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println();
  Serial.println("ESP32 SPI POLLING TEST");
  Serial.println("DMA: DISABLED");
  Serial.println("Sampling frequency: 100 Hz");
  Serial.println("SPI clock: 1 MHz");
  Serial.println("Batch size: 256 bytes");
  Serial.println("Loopback: MOSI GPIO23 -> MISO GPIO19");

  lastSample = micros();
  lastReport = millis();
}

void loop() {

  // Wait for the next 10 ms sample period
  if ((int32_t)(micros() - lastSample) < SAMPLE_INTERVAL_US) {
    return;
  }

  lastSample += SAMPLE_INTERVAL_US;

  // SPI polling transfer
  spi.beginTransaction(
    SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0)
  );

  digitalWrite(PIN_CS, LOW);

  for (int i = 0; i < BUFFER_SIZE; i++) {
    rxBuffer[i] = spi.transfer(txBuffer[i]);
  }

  digitalWrite(PIN_CS, HIGH);

  spi.endTransaction();

  // Verify received data
  bool valid = true;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (rxBuffer[i] != txBuffer[i]) {
      valid = false;
      break;
    }
  }

  if (!valid) {
    errors++;
  }

  totalBytes += BUFFER_SIZE;
  totalSamples++;

  // Report every 5 seconds
  if (millis() - lastReport >= 5000) {

    float elapsed = (millis() - lastReport) / 1000.0;

    Serial.printf(
      "Polling | Samples/sec: %.2f | Throughput: %.2f KB/s | Errors: %lu\n",
      totalSamples / elapsed,
      (totalBytes / 1024.0) / elapsed,
      (unsigned long)errors
    );

    totalBytes = 0;
    totalSamples = 0;
    errors = 0;

    lastReport = millis();
  }
}
