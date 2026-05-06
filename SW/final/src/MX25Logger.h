#ifndef MX25LOGGER_H
#define MX25LOGGER_H

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class MX25Logger {
  public:
    MX25Logger();

    bool begin(SPIClass *spi, int sck, int miso, int mosi, int cs);
    void eraseAll();

    // 버퍼에 데이터를 추가만 합니다 (플래시 쓰기 없음, 뮤텍스 보호)
    template <typename T>
    void appendData(T& data) {
      xSemaphoreTake(_bufferMutex, portMAX_DELAY);
      uint16_t len = sizeof(T);
      if (_bufferIndex + len <= BUFFER_SIZE) {
        memcpy(&_dataBuffer[_bufferIndex], &data, len);
        _bufferIndex += len;
      }
      xSemaphoreGive(_bufferMutex);
    }

    // raw 바이트 배열을 버퍼에 추가 (큐에서 받은 데이터용)
    void appendRaw(const uint8_t *data, uint16_t len) {
      xSemaphoreTake(_bufferMutex, portMAX_DELAY);
      if (_bufferIndex + len <= BUFFER_SIZE) {
        memcpy(&_dataBuffer[_bufferIndex], data, len);
        _bufferIndex += len;
      }
      xSemaphoreGive(_bufferMutex);
    }

    // 버퍼에 완성된 페이지(256바이트)가 있으면 플래시에 기록합니다 (뮤텍스 보호)
    void flushPages();

    // 남은 데이터를 모두 플래시에 기록합니다 (로깅 종료 시 호출)
    void forceFlushBuffer();

    void dumpRawBinary(Stream &out = Serial);

    // FlushTask에서 페이지가 차 있는지 확인용
    bool hasFullPage();

    uint32_t getStartAddress() { return START_ADDRESS; }
    uint32_t getCurrentAddress() { return _currentFlashAddress; }

  private:
    SPIClass *_spi;
    int _csPin;
    uint32_t _currentFlashAddress;

    static const uint16_t BUFFER_SIZE = 2048;
    uint8_t _dataBuffer[BUFFER_SIZE];
    uint16_t _bufferIndex;

    SemaphoreHandle_t _bufferMutex;
    Preferences _prefs;

    const uint32_t START_ADDRESS = 0x0000000;

    // NVS에 기록 종료 주소를 저장/복원
    void saveAddress();
    void loadAddress();

    void writePage(uint8_t *page);
    void readFlash(uint32_t addr, uint8_t *buf, uint32_t len);
    void eraseSector(uint32_t addr);
    void writeEnable();
    void waitUntilDone();
    void enter4ByteMode();
};

#endif
