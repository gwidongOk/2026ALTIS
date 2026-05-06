#include "MX25Logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define CMD_WREN         0x06
#define CMD_RDSR         0x05
#define CMD_WRITE        0x02
#define CMD_READ         0x03
#define CMD_EN4B         0xB7
#define CMD_SECTOR_ERASE 0x20
#define CMD_CHIP_ERASE   0xC7

MX25Logger::MX25Logger() {
  _spi = nullptr;
  _bufferIndex = 0;
  _currentFlashAddress = 0x1000000;
  _bufferMutex = xSemaphoreCreateMutex();
}

bool MX25Logger::begin(SPIClass *spi, int sck, int miso, int mosi, int cs) {
  _spi = spi;
  _csPin = cs;
  _currentFlashAddress = START_ADDRESS;
  _bufferIndex = 0;

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  _spi->begin(sck, miso, mosi, _csPin);
  _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  enter4ByteMode();
  vTaskDelay(pdMS_TO_TICKS(10));
  return true;
}

bool MX25Logger::hasFullPage() {
  xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  bool result = (_bufferIndex >= 256);
  xSemaphoreGive(_bufferMutex);
  return result;
}

void MX25Logger::flushPages() {
  xSemaphoreTake(_bufferMutex, portMAX_DELAY);

  while (_bufferIndex >= 256) {
    // 뮤텍스를 잡은 상태에서 256바이트를 로컬로 복사
    uint8_t page[256];
    memcpy(page, _dataBuffer, 256);

    // 버퍼에서 기록한 부분 제거
    uint16_t remaining = _bufferIndex - 256;
    if (remaining > 0) {
      memmove(_dataBuffer, &_dataBuffer[256], remaining);
    }
    _bufferIndex = remaining;

    // 뮤텍스 해제 후 플래시 쓰기 (SPI 작업 중 버퍼 잠금 방지)
    xSemaphoreGive(_bufferMutex);

    writePage(page);

    // 다시 뮤텍스 획득하여 다음 페이지 확인
    xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  }

  xSemaphoreGive(_bufferMutex);
}

void MX25Logger::writePage(uint8_t *page) {
  if (_currentFlashAddress % 4096 == 0) {
    writeEnable(); eraseSector(_currentFlashAddress); waitUntilDone();
  }
  writeEnable();
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_WRITE);
  _spi->transfer((_currentFlashAddress >> 24) & 0xFF);
  _spi->transfer((_currentFlashAddress >> 16) & 0xFF);
  _spi->transfer((_currentFlashAddress >> 8) & 0xFF);
  _spi->transfer(_currentFlashAddress & 0xFF);

  for (uint16_t i = 0; i < 256; i++) { _spi->transfer(page[i]); }
  digitalWrite(_csPin, HIGH);
  waitUntilDone();

  _currentFlashAddress += 256;
}

void MX25Logger::forceFlushBuffer() {
  // 먼저 완성된 페이지들을 플러시
  flushPages();

  xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  if (_bufferIndex == 0) {
    xSemaphoreGive(_bufferMutex);
    return;
  }

  // 남은 데이터를 로컬로 복사
  uint8_t remaining[256];
  uint16_t len = _bufferIndex;
  memcpy(remaining, _dataBuffer, len);
  _bufferIndex = 0;
  xSemaphoreGive(_bufferMutex);

  // 뮤텍스 해제 후 플래시 쓰기
  if (_currentFlashAddress % 4096 == 0) {
    writeEnable(); eraseSector(_currentFlashAddress); waitUntilDone();
  }
  writeEnable();
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_WRITE);
  _spi->transfer((_currentFlashAddress >> 24) & 0xFF);
  _spi->transfer((_currentFlashAddress >> 16) & 0xFF);
  _spi->transfer((_currentFlashAddress >> 8) & 0xFF);
  _spi->transfer(_currentFlashAddress & 0xFF);

  for (uint16_t i = 0; i < len; i++) { _spi->transfer(remaining[i]); }
  digitalWrite(_csPin, HIGH);
  waitUntilDone();

  _currentFlashAddress += len;
}

void MX25Logger::dumpRawBinary(Stream &out) {
  uint32_t readAddr = START_ADDRESS;
  uint8_t buffer[256];

  while (readAddr < _currentFlashAddress) {
    uint32_t readLen = _currentFlashAddress - readAddr;
    if (readLen > 256) readLen = 256;

    readFlash(readAddr, buffer, readLen);
    out.write(buffer, readLen);

    readAddr += readLen;
  }
}

void MX25Logger::eraseAll() {
  writeEnable();
  digitalWrite(_csPin, LOW);
  _spi->transfer(CMD_CHIP_ERASE);
  digitalWrite(_csPin, HIGH);

  digitalWrite(_csPin, LOW);
  _spi->transfer(CMD_RDSR);
  while ((_spi->transfer(0x00) & 0x01) == 1) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  digitalWrite(_csPin, HIGH);

  _currentFlashAddress = START_ADDRESS;
  _bufferIndex = 0;
}

void MX25Logger::readFlash(uint32_t addr, uint8_t *buf, uint32_t len) {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_READ);
  _spi->transfer((addr >> 24) & 0xFF); _spi->transfer((addr >> 16) & 0xFF);
  _spi->transfer((addr >> 8) & 0xFF); _spi->transfer(addr & 0xFF);
  for (uint32_t i = 0; i < len; i++) { buf[i] = _spi->transfer(0x00); }
  digitalWrite(_csPin, HIGH);
}

void MX25Logger::eraseSector(uint32_t addr) {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_SECTOR_ERASE);
  _spi->transfer((addr >> 24) & 0xFF); _spi->transfer((addr >> 16) & 0xFF);
  _spi->transfer((addr >> 8) & 0xFF); _spi->transfer(addr & 0xFF);
  digitalWrite(_csPin, HIGH);
}

void MX25Logger::writeEnable() {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_WREN); digitalWrite(_csPin, HIGH);
}

void MX25Logger::waitUntilDone() {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_RDSR);
  while ((_spi->transfer(0x00) & 0x01) == 1) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  digitalWrite(_csPin, HIGH);
}

void MX25Logger::enter4ByteMode() {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_EN4B); digitalWrite(_csPin, HIGH);
}
