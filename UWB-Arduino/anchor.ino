/*
 * [앵커 (Anchor) - 컴파일 에러 수정 및 3개 확장]
 * ★ 중요: 업로드할 때마다 ANCHOR_ID를 1, 2, 3으로 변경하세요.
 */

// ▼▼▼ 이 숫자를 1, 2, 3 중 하나로 설정해서 각각 업로드! ▼▼▼
#define ANCHOR_ID  1
// ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

#include <SPI.h>
#include "DW3000.h"
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "intel8";
const char* password = "intel00008";

// ★ 리시버(PC/우분투)의 IP 주소
const char* hostIp   = "192.168.123.42"; 
const int   hostPort = 44444;

const uint8_t PIN_SS = 4;
const uint8_t PIN_RST = 27;

WiFiUDP udp;
int loop_delay = 100;

static long long tx_poll, rx_resp, tx_final;
static int t_round1, t_reply1;

void setup() {
    Serial.begin(115200);
    
    // ID별 딜레이 설정 (충돌 방지)
    if (ANCHOR_ID == 1) loop_delay = 97;
    else if (ANCHOR_ID == 2) loop_delay = 113;
    else loop_delay = 127;

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.printf("\nAnchor %d WiFi Connected!\n", ANCHOR_ID);

    SPI.begin(18, 19, 23, 4);
    DW3000.begin();
    
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW); delay(50);
    digitalWrite(PIN_RST, HIGH); delay(200);
    
    DW3000.init();
    DW3000.setupGPIO();
    
    // ★★★ [수정] RX 설정 삭제, TX만 0으로 설정 ★★★
    DW3000.setTXAntennaDelay(0);
    
    Serial.printf("ANCHOR %d READY\n", ANCHOR_ID);
}

float measureDistance() {
    DW3000.clearSystemStatus();
    
    // 1. Poll 전송
    DW3000.ds_sendFrame(1);
    tx_poll = DW3000.readTXTimestamp();

    // 2. Response 대기 (타임아웃 30ms)
    unsigned long start = millis();
    while (!DW3000.receivedFrameSucc()) {
        if (millis() - start > 30) return -1.0; 
    }
    
    rx_resp = DW3000.readRXTimestamp();
    t_round1 = rx_resp - tx_poll;

    // 3. Final 전송
    DW3000.ds_sendFrame(3);
    tx_final = DW3000.readTXTimestamp();
    t_reply1 = tx_final - rx_resp;

    // 4. 거리 데이터 수신 대기 (타임아웃 30ms)
    start = millis();
    while (!DW3000.receivedFrameSucc()) {
        if (millis() - start > 30) return -1.0;
    }
    
    // 5. 계산
    uint32_t t_round_tag = DW3000.read(0x12, 0x04);
    uint32_t t_reply_tag = DW3000.read(0x12, 0x08);

    if (t_round_tag == 0 || t_reply_tag == 0) return -1.0;

    int ranging_time = DW3000.ds_processRTInfo(t_round1, t_reply1, t_round_tag, t_reply_tag, DW3000.getRawClockOffset());
    return DW3000.convertToCM(ranging_time) / 100.0;
}

void loop() {
    float dist = measureDistance();

    if (dist > 0.1 && dist < 50.0) { // 50m 이내 유효
        String label = "A" + String(ANCHOR_ID) + ":";
        
        Serial.print(label); Serial.println(dist);
        
        udp.beginPacket(hostIp, hostPort);
        udp.print(label + String(dist, 2));
        udp.endPacket();
    }

    delay(loop_delay);
}