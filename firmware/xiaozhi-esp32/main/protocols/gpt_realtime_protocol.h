#ifndef GPT_REALTIME_PROTOCOL_H
#define GPT_REALTIME_PROTOCOL_H

#include "protocol.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class WebSocket;

class GptRealtimeProtocol : public Protocol {
public:
    GptRealtimeProtocol();
    ~GptRealtimeProtocol();

    bool Start() override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel(bool send_goodbye = true) override;
    bool IsAudioChannelOpened() const override;
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    void SendStartListening(ListeningMode mode) override;
    void SendStopListening() override;
    void SendAbortSpeaking(AbortReason reason) override;
    void SendWakeWordDetected(const std::string& wake_word) override;
    void SendMcpMessage(const std::string& message) override;

protected:
    bool SendText(const std::string& text) override;

private:
    static constexpr EventBits_t GPT_REALTIME_SESSION_READY_EVENT = BIT0;
    static constexpr int kInputSampleRate = 24000;
    static constexpr int kOutputSampleRate = 24000;
    static constexpr int kFrameDurationMs = 60;

    std::unique_ptr<WebSocket> websocket_;
    EventGroupHandle_t event_group_handle_ = nullptr;
    void* input_opus_decoder_ = nullptr;
    void* output_opus_encoder_ = nullptr;
    void* input_resampler_ = nullptr;
    std::mutex codec_mutex_;
    std::vector<uint8_t> output_pcm_buffer_;
    bool audio_channel_opened_ = false;
    bool response_audio_started_ = false;
    bool input_audio_appended_ = false;
    std::string api_key_;
    std::string model_;
    std::string voice_;
    std::string instructions_;
    std::string url_;

    void LoadSettings();
    bool EnsureCodecs();
    bool SendSessionUpdate();
    void HandleTextMessage(const char* data, size_t len);
    void HandleAudioDelta(const cJSON* root);
    void HandleTranscriptDelta(const cJSON* root);
    void EmitTtsState(const char* state, const char* text = nullptr);
    bool DecodeInputOpus(const AudioStreamPacket& packet, std::vector<int16_t>& pcm);
    bool ResampleInputPcm(std::vector<int16_t>& pcm, int from_sample_rate);
    bool EncodeAndEmitOutputPcm(const int16_t* samples, size_t sample_count);
    static std::string Base64Encode(const uint8_t* data, size_t len);
    static std::vector<uint8_t> Base64Decode(const char* data);
};

#endif
