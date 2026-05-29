#ifndef GPT_REALTIME_PROTOCOL_H
#define GPT_REALTIME_PROTOCOL_H

#include "protocol.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <condition_variable>
#include <deque>
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
    // Wait this long after a response's first audio frame before emitting it, so
    // the (asynchronous) switch to the speaking device state completes first and
    // the playback path stops dropping the audio.
    static constexpr int kSpeakingSettleMs = 120;

    std::unique_ptr<WebSocket> websocket_;
    mutable std::mutex websocket_mutex_;
    EventGroupHandle_t event_group_handle_ = nullptr;
    void* input_opus_decoder_ = nullptr;
    void* input_resampler_ = nullptr;
    std::mutex codec_mutex_;
    std::mutex output_audio_mutex_;
    std::condition_variable output_audio_cv_;
    std::deque<std::vector<uint8_t>> output_audio_queue_;
    std::vector<uint8_t> output_pcm_buffer_;
    std::string response_transcript_;
    TaskHandle_t output_audio_task_handle_ = nullptr;
    bool audio_channel_opened_ = false;
    bool response_audio_started_ = false;
    bool response_active_ = false;
    // Set when the server signals the response is done (response.done). The output
    // audio task defers the "stop" tts event until it has played out the buffered
    // audio, so turn-taking doesn't cut playback short.
    bool response_complete_ = false;
    bool input_audio_appended_ = false;
    bool output_audio_task_running_ = false;
    bool output_new_response_ = false;
    uint32_t input_audio_packet_count_ = 0;
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
    void StartOutputAudioTask();
    void StopOutputAudioTask();
    void OutputAudioTask();
    bool DecodeInputOpus(const AudioStreamPacket& packet, std::vector<int16_t>& pcm);
    bool ResampleInputPcm(std::vector<int16_t>& pcm, int from_sample_rate);
    void EmitOutputPcm(const int16_t* samples, size_t sample_count);
    static std::string Base64Encode(const uint8_t* data, size_t len);
    static std::vector<uint8_t> Base64Decode(const char* data);
};

#endif
