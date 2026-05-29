#include "gpt_realtime_protocol.h"

#include "audio_service.h"
#include "board.h"
#include "settings.h"
#include "system_info.h"
#include "assets/lang_config.h"

#include <algorithm>
#include <cstring>
#include <cJSON.h>
#include <esp_log.h>
#include <mbedtls/base64.h>
#include <web_socket.h>

#define TAG "GPT"

#define RATE_CVT_CFG(_src_rate, _dest_rate, _channel)        \
    (esp_ae_rate_cvt_cfg_t)                                  \
    {                                                        \
        .src_rate        = (uint32_t)(_src_rate),            \
        .dest_rate       = (uint32_t)(_dest_rate),           \
        .channel         = (uint8_t)(_channel),              \
        .bits_per_sample = ESP_AUDIO_BIT16,                  \
        .complexity      = 2,                                \
        .perf_type       = ESP_AE_RATE_CVT_PERF_TYPE_SPEED,  \
    }

#define OPUS_DEC_CFG(_sample_rate, _frame_duration_ms)                                                    \
    (esp_opus_dec_cfg_t)                                                                                  \
    {                                                                                                     \
        .sample_rate    = (uint32_t)(_sample_rate),                                                       \
        .channel        = ESP_AUDIO_MONO,                                                                 \
        .frame_duration = (esp_opus_dec_frame_duration_t)AS_OPUS_GET_FRAME_DRU_ENUM(_frame_duration_ms),  \
        .self_delimited = false,                                                                          \
    }

GptRealtimeProtocol::GptRealtimeProtocol(RealtimeProvider provider)
    : provider_(provider)
{
    event_group_handle_ = xEventGroupCreate();
    server_sample_rate_ = kOutputSampleRate;
    server_frame_duration_ = kFrameDurationMs;
    // The output audio task only frames/paces PCM (no Opus encode), so it needs
    // only a small stack and can be created here. It blocks on its queue until
    // audio arrives.
    StartOutputAudioTask();
}

GptRealtimeProtocol::~GptRealtimeProtocol()
{
    CloseAudioChannel(false);
    StopOutputAudioTask();
    if (input_opus_decoder_ != nullptr) {
        esp_opus_dec_close(input_opus_decoder_);
    }
    if (input_resampler_ != nullptr) {
        esp_ae_rate_cvt_close(input_resampler_);
    }
    if (event_group_handle_ != nullptr) {
        vEventGroupDelete(event_group_handle_);
    }
}

bool GptRealtimeProtocol::Start()
{
    return true;
}

const char* GptRealtimeProtocol::ProviderName() const
{
    return provider_ == RealtimeProvider::Grok ? "Grok" : "OpenAI";
}

void GptRealtimeProtocol::LoadSettings()
{
    if (provider_ == RealtimeProvider::Grok) {
        // xAI Grok Voice Agent API. Credentials/overrides live in the NVS
        // namespace "grok"; CONFIG_GROK_* are the build-time fallbacks.
        Settings settings("grok", false);
        api_key_ = settings.GetString("api_key", CONFIG_GROK_API_KEY);
        model_ = settings.GetString("model", CONFIG_GROK_VOICE_MODEL);
        voice_ = settings.GetString("voice", CONFIG_GROK_VOICE);
        instructions_ = settings.GetString("instructions", CONFIG_GROK_INSTRUCTIONS);
        url_ = settings.GetString("url", "");
        if (url_.empty()) {
            url_ = "wss://api.x.ai/v1/realtime?model=" + model_;
        }
        // Grok server-VAD activation threshold (percent -> 0..1). 0.6 detected
        // reliably while the AFE ran continuously; detection problems came from
        // cycling the AFE off during speaking, not from this value.
        grok_vad_threshold_pct_ = settings.GetInt("vad_threshold", 60);
        grok_vad_silence_ms_ = settings.GetInt("vad_silence_ms", 700);
        grok_vad_prefix_ms_ = settings.GetInt("vad_prefix_ms", 333);
        mic_gain_ = settings.GetInt("mic_gain", 8);
        return;
    }

    Settings settings("openai", false);
    api_key_ = settings.GetString("api_key", CONFIG_OPENAI_API_KEY);
    model_ = settings.GetString("model", CONFIG_OPENAI_REALTIME_MODEL);
    voice_ = settings.GetString("voice", CONFIG_OPENAI_REALTIME_VOICE);
    instructions_ = settings.GetString("instructions", CONFIG_OPENAI_REALTIME_INSTRUCTIONS);
    url_ = settings.GetString("url", "");
    if (url_.empty()) {
        url_ = "wss://api.openai.com/v1/realtime?model=" + model_;
    }
}

bool GptRealtimeProtocol::EnsureCodecs()
{
    {
        std::lock_guard<std::mutex> lock(codec_mutex_);

        if (input_opus_decoder_ == nullptr) {
            auto dec_cfg = OPUS_DEC_CFG(16000, OPUS_FRAME_DURATION_MS);
            auto ret = esp_opus_dec_open(&dec_cfg, sizeof(dec_cfg), &input_opus_decoder_);
            if (ret != ESP_AUDIO_ERR_OK || input_opus_decoder_ == nullptr) {
                ESP_LOGE(TAG, "Failed to open input opus decoder: %d", ret);
                return false;
            }
        }

        // No output Opus encoder needed: OpenAI sends PCM and the playback path
        // accepts PCM directly (AudioStreamPacket::is_pcm), so we forward PCM
        // without a PCM->Opus->PCM round trip.

        if (input_resampler_ == nullptr) {
            auto resampler_cfg = RATE_CVT_CFG(16000, kInputSampleRate, ESP_AUDIO_MONO);
            auto ret = esp_ae_rate_cvt_open(&resampler_cfg, (esp_ae_rate_cvt_handle_t*)&input_resampler_);
            if (ret != ESP_AE_ERR_OK || input_resampler_ == nullptr) {
                ESP_LOGE(TAG, "Failed to open input resampler: %d", ret);
                return false;
            }
        }
    }

    return true;
}

bool GptRealtimeProtocol::OpenAudioChannel()
{
    LoadSettings();
    if (api_key_.empty()) {
        SetError(std::string(ProviderName()) + " API key is not configured");
        return false;
    }
    if (!EnsureCodecs()) {
        SetError(Lang::Strings::SERVER_ERROR);
        return false;
    }

    error_occurred_ = false;
    audio_channel_opened_ = false;
    input_audio_appended_ = false;
    input_audio_packet_count_ = 0;
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        response_audio_started_ = false;
        response_active_ = false;
        response_complete_ = false;
        response_transcript_.clear();
        output_pcm_buffer_.clear();
        output_audio_queue_.clear();
        output_new_response_ = false;
    }
    xEventGroupClearBits(event_group_handle_, GPT_REALTIME_SESSION_READY_EVENT);

    auto network = Board::GetInstance().GetNetwork();
    {
        std::lock_guard<std::mutex> lock(websocket_mutex_);
        websocket_ = network->CreateWebSocket(1);
    }
    if (websocket_ == nullptr) {
        SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        return false;
    }

    std::string bearer = "Bearer " + api_key_;
    websocket_->SetHeader("Authorization", bearer.c_str());
    websocket_->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    websocket_->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    websocket_->SetReceiveBufferSize(16384);
    websocket_->OnData([this](const char* data, size_t len, bool binary) {
        if (!binary) {
            HandleTextMessage(data, len);
        }
        last_incoming_time_ = std::chrono::steady_clock::now();
    });
    websocket_->OnDisconnected([this]() {
        ESP_LOGI(TAG, "Realtime websocket disconnected");
        audio_channel_opened_ = false;
        // Let the output task drain whatever is buffered and emit "stop" so the
        // device doesn't get stuck in the speaking state with a wedged turn.
        {
            std::lock_guard<std::mutex> lock(output_audio_mutex_);
            response_complete_ = true;
        }
        output_audio_cv_.notify_all();
        if (on_audio_channel_closed_ != nullptr) {
            on_audio_channel_closed_();
        }
    });

    ESP_LOGI(TAG, "Connecting to %s Realtime: %s", ProviderName(), url_.c_str());
    if (!websocket_->Connect(url_.c_str())) {
        ESP_LOGE(TAG, "Failed to connect %s Realtime, code=%d", ProviderName(), websocket_->GetLastError());
        SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        return false;
    }

    if (!SendSessionUpdate()) {
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(event_group_handle_, GPT_REALTIME_SESSION_READY_EVENT, pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(10000));
    if (!(bits & GPT_REALTIME_SESSION_READY_EVENT)) {
        SetError(Lang::Strings::SERVER_TIMEOUT);
        return false;
    }

    audio_channel_opened_ = true;
    if (on_audio_channel_opened_ != nullptr) {
        on_audio_channel_opened_();
    }
    if (on_connected_ != nullptr) {
        on_connected_();
    }
    return true;
}

void GptRealtimeProtocol::CloseAudioChannel(bool send_goodbye)
{
    (void)send_goodbye;
    audio_channel_opened_ = false;
    // Move the socket out under the lock and destroy it after releasing it, so a
    // concurrent SendText/IsAudioChannelOpened can't use a half-destroyed socket
    // and we don't hold the lock across the (possibly thread-joining) destructor.
    std::unique_ptr<WebSocket> ws;
    {
        std::lock_guard<std::mutex> lock(websocket_mutex_);
        ws = std::move(websocket_);
    }
    ws.reset();
}

bool GptRealtimeProtocol::IsAudioChannelOpened() const
{
    std::lock_guard<std::mutex> lock(websocket_mutex_);
    return websocket_ != nullptr && websocket_->IsConnected() && audio_channel_opened_ && !error_occurred_ && !IsTimeout();
}

bool GptRealtimeProtocol::SendText(const std::string& text)
{
    bool ok;
    {
        std::lock_guard<std::mutex> lock(websocket_mutex_);
        if (websocket_ == nullptr || !websocket_->IsConnected()) {
            return false;
        }
        ok = websocket_->Send(text);
    }
    if (!ok) {
        SetError(Lang::Strings::SERVER_ERROR);
        return false;
    }
    return true;
}

bool GptRealtimeProtocol::SendSessionUpdate()
{
    if (provider_ == RealtimeProvider::Grok) {
        return SendGrokSessionUpdate();
    }

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "session.update");

    cJSON* session = cJSON_AddObjectToObject(root, "session");
    cJSON_AddStringToObject(session, "type", "realtime");
    cJSON_AddStringToObject(session, "model", model_.c_str());
    cJSON_AddStringToObject(session, "instructions", instructions_.c_str());

    cJSON* output_modalities = cJSON_AddArrayToObject(session, "output_modalities");
    cJSON_AddItemToArray(output_modalities, cJSON_CreateString("audio"));

    cJSON* audio = cJSON_AddObjectToObject(session, "audio");
    cJSON* input = cJSON_AddObjectToObject(audio, "input");
    cJSON* input_format = cJSON_AddObjectToObject(input, "format");
    cJSON_AddStringToObject(input_format, "type", "audio/pcm");
    cJSON_AddNumberToObject(input_format, "rate", kInputSampleRate);

    cJSON* turn_detection = cJSON_AddObjectToObject(input, "turn_detection");
    cJSON_AddStringToObject(turn_detection, "type", "server_vad");
    cJSON_AddNumberToObject(turn_detection, "threshold", 0.5);
    cJSON_AddNumberToObject(turn_detection, "prefix_padding_ms", 300);
    cJSON_AddNumberToObject(turn_detection, "silence_duration_ms", 700);
    cJSON_AddBoolToObject(turn_detection, "create_response", true);
    cJSON_AddBoolToObject(turn_detection, "interrupt_response", true);

    cJSON* output = cJSON_AddObjectToObject(audio, "output");
    cJSON* output_format = cJSON_AddObjectToObject(output, "format");
    cJSON_AddStringToObject(output_format, "type", "audio/pcm");
    cJSON_AddNumberToObject(output_format, "rate", kOutputSampleRate);
    cJSON_AddStringToObject(output, "voice", voice_.c_str());

    char* json = cJSON_PrintUnformatted(root);
    std::string message(json);
    cJSON_free(json);
    cJSON_Delete(root);
    return SendText(message);
}

bool GptRealtimeProtocol::SendGrokSessionUpdate()
{
    // Grok's Voice Agent API takes the OpenAI beta-style session shape: voice,
    // instructions and turn_detection sit at the session top level (no
    // session.type / model / output_modalities), and the audio object only
    // carries input/output formats. The model is pinned via the URL query param.
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "session.update");

    cJSON* session = cJSON_AddObjectToObject(root, "session");
    cJSON_AddStringToObject(session, "voice", voice_.c_str());
    cJSON_AddStringToObject(session, "instructions", instructions_.c_str());

    cJSON* turn_detection = cJSON_AddObjectToObject(session, "turn_detection");
    cJSON_AddStringToObject(turn_detection, "type", "server_vad");
    cJSON_AddNumberToObject(turn_detection, "threshold", grok_vad_threshold_pct_ / 100.0);
    cJSON_AddNumberToObject(turn_detection, "prefix_padding_ms", grok_vad_prefix_ms_);
    cJSON_AddNumberToObject(turn_detection, "silence_duration_ms", grok_vad_silence_ms_);

    cJSON* audio = cJSON_AddObjectToObject(session, "audio");
    cJSON* input = cJSON_AddObjectToObject(audio, "input");
    cJSON* input_format = cJSON_AddObjectToObject(input, "format");
    cJSON_AddStringToObject(input_format, "type", "audio/pcm");
    cJSON_AddNumberToObject(input_format, "rate", kInputSampleRate);

    cJSON* output = cJSON_AddObjectToObject(audio, "output");
    cJSON* output_format = cJSON_AddObjectToObject(output, "format");
    cJSON_AddStringToObject(output_format, "type", "audio/pcm");
    cJSON_AddNumberToObject(output_format, "rate", kOutputSampleRate);

    char* json = cJSON_PrintUnformatted(root);
    std::string message(json);
    cJSON_free(json);
    cJSON_Delete(root);
    ESP_LOGD(TAG, "Grok session.update: %s", message.c_str());
    return SendText(message);
}

bool GptRealtimeProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet)
{
    // Don't feed mic audio while a response is being generated/played: with
    // server VAD + interrupt_response the device would otherwise interrupt its own
    // reply (e.g. via speaker echo).
    bool response_active;
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        response_active = response_active_;
    }
    if (!IsAudioChannelOpened() || packet == nullptr || response_active) {
        return false;
    }

    std::vector<int16_t> pcm;
    if (packet->format == AudioStreamFormat::kPcm) {
        // Mic audio arrives as raw PCM (EnableRawPcmSend), so there's no Opus to
        // decode — just reinterpret the payload.
        const int16_t* samples = reinterpret_cast<const int16_t*>(packet->payload.data());
        pcm.assign(samples, samples + packet->payload.size() / sizeof(int16_t));
    } else if (!DecodeInputOpus(*packet, pcm)) {
        return false;
    }
    // OpenAI realtime requires 24 kHz input, so resample from the 16 kHz mic rate.
    if (!ResampleInputPcm(pcm, packet->sample_rate)) {
        return false;
    }

    // Amplify the mic PCM so Grok's server VAD reliably registers speech. Clip
    // to int16. (Use a DEBUG log of the pre-gain peak when tuning grok/mic_gain.)
    if (mic_gain_ > 1) {
        for (auto& s : pcm) {
            int32_t v = (int32_t)s * mic_gain_;
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            s = (int16_t)v;
        }
    }

    auto encoded = Base64Encode(reinterpret_cast<const uint8_t*>(pcm.data()), pcm.size() * sizeof(int16_t));
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "input_audio_buffer.append");
    cJSON_AddStringToObject(root, "audio", encoded.c_str());
    char* json = cJSON_PrintUnformatted(root);
    std::string message(json);
    cJSON_free(json);
    cJSON_Delete(root);

    input_audio_appended_ = true;
    input_audio_packet_count_++;
    if (input_audio_packet_count_ == 1 || input_audio_packet_count_ % 20 == 0) {
        ESP_LOGD(TAG, "Sent input audio packet %lu (%u samples)",
                 (unsigned long)input_audio_packet_count_, (unsigned)pcm.size());
    }
    return SendText(message);
}

void GptRealtimeProtocol::SendStartListening(ListeningMode mode)
{
    (void)mode;
    input_audio_appended_ = false;
    input_audio_packet_count_ = 0;
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        response_audio_started_ = false;
        response_active_ = false;
        response_complete_ = false;
        response_transcript_.clear();
        output_pcm_buffer_.clear();
        output_audio_queue_.clear();
        output_new_response_ = false;
    }
    SendText("{\"type\":\"input_audio_buffer.clear\"}");
}

void GptRealtimeProtocol::SendStopListening()
{
    if (!input_audio_appended_) {
        return;
    }
    // Server VAD (turn_detection) commits the buffer and creates the response on its
    // own. We only flush a trailing commit; we never send response.create ourselves,
    // because racing the server's auto-created response triggers a fatal
    // "conversation already has an active response" error.
    ESP_LOGI(TAG, "Committing input audio after %lu packets", (unsigned long)input_audio_packet_count_);
    SendText("{\"type\":\"input_audio_buffer.commit\"}");
    input_audio_appended_ = false;
    input_audio_packet_count_ = 0;
}

void GptRealtimeProtocol::SendAbortSpeaking(AbortReason reason)
{
    (void)reason;
    SendText("{\"type\":\"response.cancel\"}");
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        response_audio_started_ = false;
        response_active_ = false;
        response_complete_ = false;
        response_transcript_.clear();
        output_pcm_buffer_.clear();
        output_audio_queue_.clear();
        output_new_response_ = false;
    }
    EmitTtsState("stop");
}

void GptRealtimeProtocol::SendWakeWordDetected(const std::string& wake_word)
{
    (void)wake_word;
}

void GptRealtimeProtocol::SendMcpMessage(const std::string& message)
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "conversation.item.create");
    cJSON* item = cJSON_AddObjectToObject(root, "item");
    cJSON_AddStringToObject(item, "type", "message");
    cJSON_AddStringToObject(item, "role", "user");
    cJSON* content = cJSON_AddArrayToObject(item, "content");
    cJSON* part = cJSON_CreateObject();
    cJSON_AddStringToObject(part, "type", "input_text");
    cJSON_AddStringToObject(part, "text", message.c_str());
    cJSON_AddItemToArray(content, part);
    char* json = cJSON_PrintUnformatted(root);
    SendText(json);
    cJSON_free(json);
    cJSON_Delete(root);
    SendText("{\"type\":\"response.create\"}");
}

void GptRealtimeProtocol::HandleTextMessage(const char* data, size_t len)
{
    std::string message(data, len);
    cJSON* root = cJSON_Parse(message.c_str());
    if (root == nullptr) {
        ESP_LOGW(TAG, "Failed to parse realtime event");
        return;
    }

    auto type = cJSON_GetObjectItem(root, "type");
    if (cJSON_IsString(type)) {
        ESP_LOGD(TAG, "Realtime event: %s", type->valuestring);
        if (strcmp(type->valuestring, "session.created") == 0 || strcmp(type->valuestring, "session.updated") == 0) {
            xEventGroupSetBits(event_group_handle_, GPT_REALTIME_SESSION_READY_EVENT);
        } else if (strcmp(type->valuestring, "response.output_audio.delta") == 0 ||
                   strcmp(type->valuestring, "response.audio.delta") == 0) {
            HandleAudioDelta(root);
        } else if (strcmp(type->valuestring, "response.output_audio.done") == 0 ||
                   strcmp(type->valuestring, "response.audio.done") == 0 ||
                   strcmp(type->valuestring, "response.done") == 0) {
            // The server finishes sending faster than real time. Don't stop now or
            // we'd cut playback short; mark the response complete and let the output
            // audio task emit "stop" once it has played out the buffered audio.
            // (A text-only response has no audio to drain, so end it immediately.)
            bool text_only_stop = false;
            std::string final_text;
            {
                std::lock_guard<std::mutex> lock(output_audio_mutex_);
                final_text = response_transcript_;  // full text (display updates are throttled)
                if (response_audio_started_) {
                    response_complete_ = true;
                } else if (response_active_ && !response_complete_) {
                    response_active_ = false;
                    response_transcript_.clear();
                    text_only_stop = true;
                }
            }
            output_audio_cv_.notify_all();
            // Make sure the complete transcript is shown (throttling may have skipped
            // the last deltas); the bubble dedupes if it's unchanged.
            if (!final_text.empty()) {
                EmitTtsState("sentence_start", final_text.c_str());
            }
            if (text_only_stop) {
                EmitTtsState("stop");
            }
            // The whole reply is in; tell the display it can start looping the text.
            // response.done is the single terminal event.
            if (strcmp(type->valuestring, "response.done") == 0) {
                EmitTtsState("complete");
            }
        } else if (strcmp(type->valuestring, "response.output_audio_transcript.delta") == 0 ||
                   strcmp(type->valuestring, "response.audio_transcript.delta") == 0 ||
                   strcmp(type->valuestring, "response.output_text.delta") == 0) {
            HandleTranscriptDelta(root);
        } else if (strcmp(type->valuestring, "error") == 0) {
            auto error = cJSON_GetObjectItem(root, "error");
            auto msg = cJSON_GetObjectItem(error, "message");
            ESP_LOGE(TAG, "Realtime error: %s", cJSON_IsString(msg) ? msg->valuestring : "(no message)");
            SetError(cJSON_IsString(msg) ? msg->valuestring : Lang::Strings::SERVER_ERROR);
        }
    }

    cJSON_Delete(root);
}

void GptRealtimeProtocol::HandleAudioDelta(const cJSON* root)
{
    auto delta = cJSON_GetObjectItem(root, "delta");
    if (!cJSON_IsString(delta)) {
        return;
    }

    auto bytes = Base64Decode(delta->valuestring);
    if (bytes.empty()) {
        return;
    }

    bool started_now = false;
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        if (!response_audio_started_) {
            response_audio_started_ = true;
            response_active_ = true;
            output_new_response_ = true;
            started_now = true;
        }
        output_audio_queue_.push_back(std::move(bytes));
    }
    output_audio_cv_.notify_all();
    if (started_now) {
        EmitTtsState("start");
    }
}

void GptRealtimeProtocol::HandleTranscriptDelta(const cJSON* root)
{
    auto delta = cJSON_GetObjectItem(root, "delta");
    if (!cJSON_IsString(delta) || delta->valuestring[0] == '\0') {
        return;
    }
    // The realtime API streams the transcript in small deltas, but SetChatMessage
    // replaces the whole bubble (a full O(n) re-layout). Accumulate every delta, but
    // only push to the display a few times per second so the rendering doesn't starve
    // the audio tasks on long replies. The complete text is emitted once on done.
    std::string snapshot;
    bool emit = false;
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        // The transcript can start before the first audio frame; mark the response
        // active so we stop streaming mic audio into it (see SendAudio).
        response_active_ = true;
        response_transcript_ += delta->valuestring;

        uint32_t now = xTaskGetTickCount();
        if (now - last_transcript_tick_ >= pdMS_TO_TICKS(kTranscriptEmitIntervalMs)) {
            last_transcript_tick_ = now;
            snapshot = response_transcript_;
            emit = true;
        }
    }
    if (emit) {
        EmitTtsState("sentence_start", snapshot.c_str());
    }
}

void GptRealtimeProtocol::EmitTtsState(const char* state, const char* text)
{
    if (on_incoming_json_ == nullptr) {
        return;
    }

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "tts");
    cJSON_AddStringToObject(root, "state", state);
    if (text != nullptr) {
        cJSON_AddStringToObject(root, "text", text);
    }
    on_incoming_json_(root);
    cJSON_Delete(root);
}

void GptRealtimeProtocol::StartOutputAudioTask()
{
    std::lock_guard<std::mutex> lock(output_audio_mutex_);
    if (output_audio_task_handle_ != nullptr) {
        return;
    }
    output_audio_task_running_ = true;
    auto ret = xTaskCreate([](void* arg) {
        GptRealtimeProtocol* protocol = (GptRealtimeProtocol*)arg;
        protocol->OutputAudioTask();
        vTaskDelete(NULL);
    }, "gpt_audio", 1024 * 4, this, 2, &output_audio_task_handle_);
    if (ret != pdPASS) {
        output_audio_task_running_ = false;
        ESP_LOGE(TAG, "Failed to create gpt_audio task (out of memory?)");
    }
}

void GptRealtimeProtocol::StopOutputAudioTask()
{
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        output_audio_task_running_ = false;
        output_audio_queue_.clear();
        output_pcm_buffer_.clear();
    }
    output_audio_cv_.notify_all();
    while (output_audio_task_handle_ != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void GptRealtimeProtocol::OutputAudioTask()
{
    const size_t frame_bytes = kOutputSampleRate * kFrameDurationMs / 1000 * sizeof(int16_t);
    const TickType_t frame_ticks = pdMS_TO_TICKS(kFrameDurationMs);
    // Keep a few frames of playback lead so brief scheduling jitter doesn't starve
    // the speaker, while staying far below the decoder queue limit.
    const TickType_t lead_ticks = frame_ticks * 3;
    TickType_t next_emit = xTaskGetTickCount();

    // Lightweight per-response playback health, logged as a single line per turn
    // (see finalize). underruns counts mid-response stalls long enough to drain
    // the downstream cushion (network can't keep up); max_late tracks how far
    // behind real-time pacing fell (CPU contention / accumulated stalls).
    uint32_t pb_frames = 0;
    uint32_t pb_underruns = 0;
    int32_t pb_max_late_ms = 0;
    bool pb_playing = false;

    while (true) {
        std::vector<uint8_t> bytes;
        bool new_response = false;
        bool finalize = false;
        {
            std::unique_lock<std::mutex> lock(output_audio_mutex_);
            const bool may_starve = pb_playing && output_audio_queue_.empty() && !response_complete_;
            const TickType_t wait_start = may_starve ? xTaskGetTickCount() : 0;
            output_audio_cv_.wait(lock, [this]() {
                return !output_audio_task_running_ || !output_audio_queue_.empty() || response_complete_;
            });
            if (may_starve && !output_audio_queue_.empty() &&
                (xTaskGetTickCount() - wait_start) >= lead_ticks) {
                pb_underruns++;
            }
            if (!output_audio_task_running_) {
                break;
            }
            if (!output_audio_queue_.empty()) {
                bytes = std::move(output_audio_queue_.front());
                output_audio_queue_.pop_front();
                new_response = output_new_response_;
                output_new_response_ = false;
            } else {
                // Queue drained and the server marked the response complete: the
                // whole turn has been emitted to the playback pipeline, so end it.
                finalize = true;
            }
        }

        if (finalize) {
            {
                std::lock_guard<std::mutex> lock(output_audio_mutex_);
                output_pcm_buffer_.clear();
                response_complete_ = false;
                response_active_ = false;
                response_audio_started_ = false;
                response_transcript_.clear();
            }
            if (pb_playing) {
                ESP_LOGD(TAG, "Playback health: %u frames, %u underruns, max %dms behind",
                         (unsigned)pb_frames, (unsigned)pb_underruns, (int)pb_max_late_ms);
                pb_playing = false;
            }
            EmitTtsState("stop");
            continue;
        }

        if (new_response) {
            // OpenAI streams a whole turn of audio faster than real time. The device
            // only accepts playback once it has switched to the speaking state (an
            // async transition) and silently drops frames when its decode queue is
            // full, so wait for that switch, then build a small lead and pace the
            // rest to real time.
            vTaskDelay(pdMS_TO_TICKS(kSpeakingSettleMs));
            next_emit = xTaskGetTickCount() - lead_ticks;
            pb_frames = 0;
            pb_underruns = 0;
            pb_max_late_ms = 0;
            pb_playing = true;
        }

        {
            std::lock_guard<std::mutex> lock(output_audio_mutex_);
            output_pcm_buffer_.insert(output_pcm_buffer_.end(), bytes.begin(), bytes.end());
        }

        while (true) {
            std::vector<int16_t> frame;
            {
                std::lock_guard<std::mutex> lock(output_audio_mutex_);
                if (output_pcm_buffer_.size() < frame_bytes) {
                    break;
                }
                auto* samples = reinterpret_cast<const int16_t*>(output_pcm_buffer_.data());
                frame.assign(samples, samples + frame_bytes / sizeof(int16_t));
                output_pcm_buffer_.erase(output_pcm_buffer_.begin(), output_pcm_buffer_.begin() + frame_bytes);
            }

            int32_t wait = (int32_t)(next_emit - xTaskGetTickCount());
            if (wait > 0) {
                vTaskDelay((TickType_t)wait);
            } else {
                int32_t late_ms = (int32_t)(-wait) * portTICK_PERIOD_MS;
                if (late_ms > pb_max_late_ms) {
                    pb_max_late_ms = late_ms;
                }
            }
            next_emit += frame_ticks;

            EmitOutputPcm(frame.data(), frame.size());
            pb_frames++;
        }
    }

    output_audio_task_handle_ = nullptr;
}

bool GptRealtimeProtocol::DecodeInputOpus(const AudioStreamPacket& packet, std::vector<int16_t>& pcm)
{
    std::lock_guard<std::mutex> lock(codec_mutex_);
    int sample_rate = packet.sample_rate == 0 ? 16000 : packet.sample_rate;
    int frame_duration = packet.frame_duration == 0 ? OPUS_FRAME_DURATION_MS : packet.frame_duration;
    size_t samples = sample_rate / 1000 * frame_duration;
    pcm.resize(samples);

    esp_audio_dec_in_raw_t raw = {
        .buffer = const_cast<uint8_t*>(packet.payload.data()),
        .len = (uint32_t)packet.payload.size(),
        .consumed = 0,
        .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
    };
    esp_audio_dec_out_frame_t out = {
        .buffer = reinterpret_cast<uint8_t*>(pcm.data()),
        .len = (uint32_t)(pcm.size() * sizeof(int16_t)),
        .decoded_size = 0,
    };
    esp_audio_dec_info_t info = {};
    auto ret = esp_opus_dec_decode(input_opus_decoder_, &raw, &out, &info);
    if (ret != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "Failed to decode input opus: %d", ret);
        return false;
    }
    pcm.resize(out.decoded_size / sizeof(int16_t));
    return true;
}

bool GptRealtimeProtocol::ResampleInputPcm(std::vector<int16_t>& pcm, int from_sample_rate)
{
    if (from_sample_rate == 0 || from_sample_rate == kInputSampleRate) {
        return true;
    }
    if (from_sample_rate != 16000) {
        ESP_LOGW(TAG, "Unsupported input sample rate for GPT resampler: %d", from_sample_rate);
        return false;
    }

    uint32_t max_output_samples = 0;
    esp_ae_rate_cvt_get_max_out_sample_num((esp_ae_rate_cvt_handle_t)input_resampler_, pcm.size(), &max_output_samples);
    std::vector<int16_t> resampled(max_output_samples);
    uint32_t actual_output_samples = max_output_samples;
    auto ret = esp_ae_rate_cvt_process((esp_ae_rate_cvt_handle_t)input_resampler_, (esp_ae_sample_t)pcm.data(),
                                       pcm.size(), (esp_ae_sample_t)resampled.data(), &actual_output_samples);
    if (ret != ESP_AE_ERR_OK) {
        ESP_LOGE(TAG, "Failed to resample input pcm: %d", ret);
        return false;
    }
    resampled.resize(actual_output_samples);
    pcm = std::move(resampled);
    return true;
}

void GptRealtimeProtocol::EmitOutputPcm(const int16_t* samples, size_t sample_count)
{
    if (on_incoming_audio_ == nullptr) {
        return;
    }
    // OpenAI realtime delivers PCM, and the device playback path can take PCM
    // directly (AudioStreamFormat::kPcm), so just forward it instead of doing a
    // wasteful PCM->Opus->PCM round trip. This also keeps this task's stack tiny.
    std::vector<uint8_t> pcm_bytes(sample_count * sizeof(int16_t));
    memcpy(pcm_bytes.data(), samples, pcm_bytes.size());

    on_incoming_audio_(std::make_unique<AudioStreamPacket>(AudioStreamPacket{
        .sample_rate = kOutputSampleRate,
        .frame_duration = kFrameDurationMs,
        .timestamp = 0,
        .format = AudioStreamFormat::kPcm,
        .payload = std::move(pcm_bytes),
    }));
}

std::string GptRealtimeProtocol::Base64Encode(const uint8_t* data, size_t len)
{
    size_t output_len = 0;
    mbedtls_base64_encode(nullptr, 0, &output_len, data, len);
    std::string output(output_len, '\0');
    if (mbedtls_base64_encode(reinterpret_cast<unsigned char*>(output.data()), output.size(), &output_len, data, len) != 0) {
        return {};
    }
    output.resize(output_len);
    return output;
}

std::vector<uint8_t> GptRealtimeProtocol::Base64Decode(const char* data)
{
    size_t input_len = strlen(data);
    size_t output_len = 0;
    mbedtls_base64_decode(nullptr, 0, &output_len, reinterpret_cast<const unsigned char*>(data), input_len);
    std::vector<uint8_t> output(output_len);
    if (mbedtls_base64_decode(output.data(), output.size(), &output_len, reinterpret_cast<const unsigned char*>(data), input_len) != 0) {
        return {};
    }
    output.resize(output_len);
    return output;
}
