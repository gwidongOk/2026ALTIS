#ifndef BLE_H
#define BLE_H

#include <Arduino.h>

// BLE 및 통신 초기화
void initBLE(const char *deviceName);

// 시리얼/블루투스 입력을 날것(Raw) 그대로 반환
String getIncomingRaw();

// 응답 메시지 전송
void sendResponse(const char *msg);

#endif