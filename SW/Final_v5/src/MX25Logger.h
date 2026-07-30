#ifndef MX25LOGGER_H
#define MX25LOGGER_H

#include <Arduino.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_heap_caps.h>

#include "sensor_data.h"

// Pre-trigger ring buffer (PSRAM): keeps recent sensor/state/event records while
// armed and throughout launch verification. A confirmed launch backfills those
// records into the flash log; a rejected candidate discards them.
class PreTriggerBuffer {
  public:
    enum RecordType : uint8_t {
      RECORD_IMU   = 1,
      RECORD_BARO  = 2,
      RECORD_STATE = 3,
      RECORD_EVENT = 4
    };

    struct EventRecord {
      uint32_t timestamp;
      uint8_t phase;
      uint8_t eventId;
    };

    struct Record {
      uint8_t type;
      union {
        Raw_imu       imu;
        Raw_press     baro;
        State_nominal state;
        EventRecord   event;
      };
    };

    bool begin(size_t capacity);
    void clear();
    void freeze();
    void pushImu(const Raw_imu &raw);
    void pushBaro(const Raw_press &baro);
    void pushState(const State_nominal &state);
    void pushEvent(FlightPhase phase, uint8_t eventId, uint32_t timestamp);
    size_t size();
    bool getOldest(size_t index, Record &out);

  private:
    Record *_records = nullptr;
    size_t _capacity = 0;
    size_t _head = 0;
    size_t _count = 0;
    bool _frozen = false;
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    void push(const Record &record);
};

class MX25Logger {
  public:
    MX25Logger();

    bool begin(SPIClass *spi, int sck, int miso, int mosi, int cs, SemaphoreHandle_t spiMutex = NULL);
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

    // ============================================================
    // Typed log entry points — packet layout lives in MX25Logger.cpp.
    // To change the on-flash format, edit only the corresponding body.
    // ============================================================
    void logImu  (const Raw_imu       &raw);
    void logBaro (const Raw_press     &p);
    void logState(const State_nominal &nom);
    void logEvent(FlightPhase phase, uint8_t eventId);

    // Core 0 uses this only while committing the frozen pre-trigger buffer.
    // It bypasses the live queue so older samples reach flash first.
    bool appendPreTriggerRecord(const PreTriggerBuffer::Record &record);

    // Drain in-RAM queue → flash (called from FlushTask)
    void serviceFlush();

    // Logging gate
    void setEnabled(bool e) { _enabled = e; }
    bool isEnabled() const  { return _enabled; }

  private:
    SPIClass *_spi;
    int _csPin;
    SemaphoreHandle_t _spiMutex;
    uint32_t _currentFlashAddress;

    static const uint16_t BUFFER_SIZE = 2048;
    uint8_t _dataBuffer[BUFFER_SIZE];
    uint16_t _bufferIndex;

    SemaphoreHandle_t _bufferMutex;

    const uint32_t START_ADDRESS = 0x0000000;
    // MX25L25645GM2I-08G : 256Mbit = 32MB
    static const uint32_t MAX_ADDRESS = 0x02000000;

    // ---- Typed log queue (in-RAM staging from sensor tasks → FlushTask) ----
    static const uint8_t  ITEM_MAX_SIZE = 64;   // largest packet (state_pkt = 47 B)
    static const size_t   SERVICE_BATCH_ITEMS = 32;
    static const size_t   PSRAM_RESERVE_BYTES = 2 * 1024 * 1024;
    // Use nearly all available PSRAM after preserving PSRAM_RESERVE_BYTES.
    // 100,000 is only a ceiling; begin() still scales this down to free PSRAM.
    static const size_t   MAX_QUEUE_ITEMS = 100000;
    struct Item {
      uint8_t data[ITEM_MAX_SIZE];
      uint8_t len;
    };
    QueueHandle_t _queue;
    StaticQueue_t _queueControl;
    uint8_t *_queueStorage;
    size_t _queueLength;
    volatile uint32_t _droppedItems;
    volatile bool _enabled;
    void _push(const void *pkt, uint8_t len);

    // NVS에 기록 종료 주소를 저장/복원
    void writePage(uint8_t *page);
    void readFlash(uint32_t addr, uint8_t *buf, uint32_t len);
    void eraseSector(uint32_t addr);
    void writeEnable();
    void waitUntilDone();
    void enter4ByteMode();
};

#endif
