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

// NVS 키
static const char* NVS_NAMESPACE = "mx25log";
static const char* NVS_KEY_ADDR  = "endAddr";

MX25Logger::MX25Logger() {
  _spi = nullptr;
  _bufferIndex = 0;
  _currentFlashAddress = 0x0000000;
  _bufferMutex = xSemaphoreCreateMutex();
}

bool MX25Logger::begin(SPIClass *spi, int sck, int miso, int mosi, int cs) {
  _spi = spi;
  _csPin = cs;
  _bufferIndex = 0;

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  _spi->begin(sck, miso, mosi, _csPin);

  enter4ByteMode();
  vTaskDelay(pdMS_TO_TICKS(10));

  // NVS에서 이전 기록 종료 주소 복원
  loadAddress();

  return true;
}

// ============================================================
// NVS 영속 저장
// ============================================================
void MX25Logger::saveAddress() {
  _prefs.begin(NVS_NAMESPACE, false);
  _prefs.putUInt(NVS_KEY_ADDR, _currentFlashAddress);
  _prefs.end();
}

void MX25Logger::loadAddress() {
  _prefs.begin(NVS_NAMESPACE, true);   // read-only
  _currentFlashAddress = _prefs.getUInt(NVS_KEY_ADDR, START_ADDRESS);
  _prefs.end();

  // 유효성 검증: START_ADDRESS 미만이면 리셋
  if (_currentFlashAddress < START_ADDRESS) {
    _currentFlashAddress = START_ADDRESS;
  }

  Serial.printf("Flash resume addr: 0x%08X (%u bytes logged)\n",
                _currentFlashAddress,
                _currentFlashAddress - START_ADDRESS);
}

// ============================================================
// 버퍼/페이지 관리
// ============================================================
bool MX25Logger::hasFullPage() {
  xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  bool result = (_bufferIndex >= 256);
  xSemaphoreGive(_bufferMutex);
  return result;
}

void MX25Logger::flushPages() {
  xSemaphoreTake(_bufferMutex, portMAX_DELAY);

  while (_bufferIndex >= 256) {
    uint8_t page[256];
    memcpy(page, _dataBuffer, 256);

    uint16_t remaining = _bufferIndex - 256;
    if (remaining > 0) {
      memmove(_dataBuffer, &_dataBuffer[256], remaining);
    }
    _bufferIndex = remaining;

    xSemaphoreGive(_bufferMutex);

    writePage(page);

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
  flushPages();

  xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  if (_bufferIndex == 0) {
    xSemaphoreGive(_bufferMutex);
    // 주소 저장 (STOP 시 호출되므로 여기서 영속화)
    saveAddress();
    return;
  }

  uint8_t remaining[256];
  uint16_t len = _bufferIndex;
  memcpy(remaining, _dataBuffer, len);
  _bufferIndex = 0;
  xSemaphoreGive(_bufferMutex);

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

  // 주소 NVS에 저장 (리부팅 후에도 복원 가능)
  saveAddress();
}

// ============================================================
// 읽기/덤프
// ============================================================
void MX25Logger::dumpRawBinary(Stream &out) {
  uint32_t readAddr = START_ADDRESS;
  uint8_t buffer[256];
  uint32_t totalBytes = _currentFlashAddress - START_ADDRESS;

  while (readAddr < _currentFlashAddress) {
    uint32_t readLen = _currentFlashAddress - readAddr;
    if (readLen > 256) readLen = 256;

    readFlash(readAddr, buffer, readLen);
    out.write(buffer, readLen);

    readAddr += readLen;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  out.flush();
}

// ============================================================
// 전체 삭제
// ============================================================
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

  // NVS도 리셋
  saveAddress();
}

// ============================================================
// 저수준 Flash 명령
// ============================================================
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
