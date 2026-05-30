#ifndef GPT_REALTIME_PROTOCOL_H
#define GPT_REALTIME_PROTOCOL_H

#include "protocol.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

class WebSocket;

// The OpenAI Realtime and xAI Grok Voice Agent APIs speak the same WebSocket
// protocol; only the endpoint, auth/NVS namespace and session.update shape
// differ. This single protocol implementation serves both, selected here.
enum class RealtimeProvider {
    OpenAi,
    Grok,
};

class GptRealtimeProtocol : public Protocol {
public:
    explicit GptRealtimeProtocol(RealtimeProvider provider = RealtimeProvider::OpenAi);
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
    bool SupportsImageAttachment() const override;
    bool SendImageMessage(const std::string& data_uri, const std::string& text = "") override;

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
    // Throttle transcript display updates. Re-rendering the whole growing text on
    // every delta is O(n) per delta and, on long replies, starves the audio tasks
    // of CPU (playback stalls until rendering settles). Accumulate every delta but
    // push to the display at most this often; the final text is emitted on done.
    static constexpr int kTranscriptEmitIntervalMs = 150;

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
    uint32_t last_transcript_tick_ = 0;
    bool input_audio_appended_ = false;
    bool output_audio_task_running_ = false;
    bool output_new_response_ = false;
    // True while a camera function tool is running (head move + capture + touch OK/NG
    // + vision HTTP). Suppresses mic streaming so server-VAD can't open a competing
    // turn during the capture. Cleared as soon as the tool finishes (function output
    // sent). A failsafe timeout in SendAudio re-enables the mic if the tool hangs
    // (e.g. the user never touches OK/NG) so input can never mute forever.
    bool function_active_ = false;
    uint32_t function_active_since_ms_ = 0;
    static constexpr uint32_t kFunctionMicMuteMaxMs = 30000;
    uint32_t input_audio_packet_count_ = 0;
    std::map<std::string, std::string> function_call_names_;
    std::set<std::string> dispatched_call_ids_;
    std::mutex function_call_mutex_;
    RealtimeProvider provider_ = RealtimeProvider::OpenAi;
    std::string api_key_;
    std::string model_;
    std::string voice_;
    std::string instructions_;
    std::string url_;
    std::string vision_model_;
    std::string vision_detail_;
    int vision_max_output_tokens_ = 160;
    // Grok server-VAD tuning (NVS-overridable in the "grok" namespace). Defaults
    // match xAI's documented defaults; OpenAI uses its own fixed values above.
    // Threshold is stored as an integer percent (85 -> 0.85) since NVS has no float.
    int grok_vad_threshold_pct_ = 60;
    int grok_vad_silence_ms_ = 700;
    int grok_vad_prefix_ms_ = 333;
    // Digital gain applied to mic PCM before sending (NVS: grok/mic_gain). The
    // mic level reaching Grok is low, so its VAD ignores normal speech; amplify
    // it. 1 = unchanged (OpenAI path).
    int mic_gain_ = 1;

    const char* ProviderName() const;

    void LoadSettings();
    bool EnsureCodecs();
    bool SendSessionUpdate();
    bool SendGrokSessionUpdate();
    void AddStackChanFunctionTools(cJSON* session);
    void HandleTextMessage(const char* data, size_t len);
    void RememberFunctionCallName(const cJSON* root);
    void HandleFunctionCall(const cJSON* root);
    bool HandleFunctionCallsFromResponseDone(const cJSON* root);
    void DispatchFunctionCall(const std::string& call_id, const std::string& function_name,
                              const std::string& arguments_json);
    void HandleAudioDelta(const cJSON* root);
    void HandleTranscriptDelta(const cJSON* root);
    void EmitTtsState(const char* state, const char* text = nullptr);
    bool SendUserTextItem(const std::string& text, bool create_response);
    bool ExecuteStackChanCameraTool(const char* name, const cJSON* args, std::string& output);
    bool CaptureImageForFunctionTool(bool move_head, int yaw, int pitch, int speed, int settle_ms,
                                     const std::string& question, std::string& output);
    bool DescribeImageWithGrok(const std::string& data_uri, std::string& description);
    std::string ExtractTextFromJson(const cJSON* root) const;
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
