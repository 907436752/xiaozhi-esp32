#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_now.h"
#include "lamp_protocol.h"

class SmartLampBridge {
public:
    static SmartLampBridge& GetInstance();

    void Start();

    bool IsOnline(uint32_t timeout_ms = 3000);
    bool GetLastStatus(s3_status_packet_t* out, uint32_t timeout_ms = 3000);

    void SendCommand(uint8_t mode, uint8_t brightness, uint8_t blink);
    void RegisterMcpTools();

    void AdjustBrightnessByVoice(int delta);
    void SetBrightnessByVoice(uint8_t brightness);
    void TurnOffTemporarilyByVoice();
    void ClearVoiceOverride();
    void SendLargeDisplay(uint8_t page,
                          uint8_t level,
                          const char* title,
                          const char* line1,
                          const char* line2,
                          const char* line3);

private:
    SmartLampBridge() = default;

    static void RecvCallback(const esp_now_recv_info_t* recv_info,
                             const uint8_t* data,
                             int data_len);

    static void RecvTask(void* arg);
    static void CommandTask(void* arg);

    void HandleStatusPacket(const s3_status_packet_t& packet);
    void AddBroadcastPeer();

    bool started_ = false;
};