#include "BLE.h"
#include <NimBLEDevice.h>

// BLE→메인 명령 전달용 FreeRTOS 큐 (레이스 컨디션 방지)
static QueueHandle_t bleRxQueue = nullptr;
static NimBLECharacteristic *pTxChar = nullptr;

// ============================================================
// NimBLE 콜백
// ============================================================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &connInfo) override {
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &connInfo, int reason) override {
    NimBLEDevice::startAdvertising();   // 자동 재광고
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
    std::string val = pChar->getValue();
    if (val.empty() || !bleRxQueue) return;

    // 고정 크기 버퍼에 복사하여 큐로 전달 (ISR-safe 아닌 일반 콜백이므로 portMAX_DELAY 사용 가능)
    char buf[32] = {0};
    size_t len = val.size();
    if (len > 31) len = 31;
    memcpy(buf, val.c_str(), len);
    xQueueSend(bleRxQueue, buf, pdMS_TO_TICKS(10));
  }
};

// ============================================================
// 초기화
// ============================================================
void initBLE(const char *deviceName) {
  // 명령 큐 생성 (최대 4개, 각 32바이트)
  bleRxQueue = xQueueCreate(4, 32);

  NimBLEDevice::init(deviceName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(128);

  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Nordic UART Service (NUS)
  NimBLEService *pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

  // TX: 보드→앱 (Notify) — NimBLE가 CCCD 자동 관리
  pTxChar = pService->createCharacteristic(
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
    NIMBLE_PROPERTY::NOTIFY
  );

  // RX: 앱→보드 (Write)
  NimBLECharacteristic *pRxChar = pService->createCharacteristic(
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxChar->setCallbacks(new RxCallbacks());

  // NimBLE v2.x: 서비스는 서버 start() 시 자동 시작
  pServer->start();

  // 광고 설정
  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->reset();                      // 기존 광고 데이터 초기화
  pAdv->setName(deviceName);          // 광고 패킷에 디바이스 이름 포함
  pAdv->addServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  pAdv->enableScanResponse(true);
  pAdv->setMinInterval(0x20);         // 20ms — 빠른 검색
  pAdv->setMaxInterval(0x40);         // 40ms

  bool ok = pAdv->start(0);           // 0 = 무기한 광고
  Serial.printf("BLE advertising %s (%s)\n", ok ? "started" : "FAILED", deviceName);
}

// ============================================================
// 입력 수신 (Serial + BLE 큐에서 thread-safe하게 가져옴)
// ============================================================
String getIncomingRaw() {
  // Serial 우선
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    return s;
  }

  // BLE 큐에서 수신
  if (bleRxQueue) {
    char buf[32];
    if (xQueueReceive(bleRxQueue, buf, 0) == pdTRUE) {
      String s = String(buf);
      s.trim();
      return s;
    }
  }

  return "";
}

// ============================================================
// 응답 전송 (Serial + BLE Notify)
// ============================================================
void sendResponse(const char *msg) {
  if (pTxChar) {
    pTxChar->setValue((const uint8_t *)msg, strlen(msg));
    pTxChar->notify();
  }
}
