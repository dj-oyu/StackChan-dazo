#include "gpt_realtime_protocol.h"

#include "audio_service.h"
#include "board.h"
#include "settings.h"
#include "system_info.h"
#include "assets/lang_config.h"
#include "application.h"
#include "display.h"
#include "lvgl_display.h"
#include <hal/hal.h>

#include <algorithm>
#include <cstring>
#include <cJSON.h>
#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_crt_bundle.h>
#include <mbedtls/base64.h>
#include <web_socket.h>
#include <freertos/semphr.h>
#include <stackchan/stackchan.h>
#include <hal/board/hal_bridge.h>
#include <assets/assets.h>
#include <mcp_server.h>
#include <cstdio>

#define TAG "GPT"

// Grok/OpenAI function names must match ^[a-zA-Z0-9_-]+$, but MCP tool names use
// dots (e.g. "self.robot.set_head_angles"). Map dots to underscores for the wire
// name; resolution back to the MCP tool compares this sanitized form so the
// mapping never needs to be stored.
static std::string SanitizeToolName(const std::string& name)
{
    std::string out = name;
    for (auto& c : out) {
        if (c == '.') {
            c = '_';
        }
    }
    return out;
}

// Resolve a sanitized wire name back to the exact MCP tool name, or "" if none.
static std::string ResolveMcpToolName(const std::string& wire_name)
{
    for (McpTool* tool : McpServer::GetInstance().tools()) {
        if (tool->user_only()) {
            continue;
        }
        if (SanitizeToolName(tool->name()) == wire_name) {
            return tool->name();
        }
    }
    return "";
}

static bool IsCameraToolName(const std::string& mcp_name)
{
    return mcp_name == "self.camera.take_photo" || mcp_name == "self.camera.look_and_take_photo";
}

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
    UnregisterCameraExplainDelegate();
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
        vision_model_ = settings.GetString("vision_model", "grok-4.3");
        vision_detail_ = settings.GetString("vision_detail", "low");
        vision_max_output_tokens_ = settings.GetInt("vision_max_output_tokens", 160);
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

    RegisterCameraExplainDelegate();

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
        function_active_ = false;
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
    UnregisterCameraExplainDelegate();
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

void GptRealtimeProtocol::RegisterCameraExplainDelegate()
{
    auto* camera = Board::GetInstance().GetCamera();
    if (camera == nullptr) {
        camera_explain_delegate_registered_ = false;
        return;
    }

    // Route the camera's "describe the captured frame" step through this protocol's
    // vision path. The MCP camera tools own the capture/confirm flow and call
    // camera->Explain(); this delegate is cleared when this channel/protocol ends.
    camera->SetExplainDelegate([this, camera](const std::string& question, std::string& description) -> bool {
        std::string data_uri;
        if (!camera->EncodeToJpegDataUri(data_uri, 80)) {
            return false;
        }
        if (provider_ == RealtimeProvider::OpenAi) {
            std::string note =
                "The user approved this camera photo for the current request. Use it as visual context to answer: " +
                question;
            if (!SendImageMessage(data_uri, note)) {
                return false;
            }
            description = "The approved camera photo was attached to the conversation as visual context.";
            return true;
        }

        if (auto* display = Board::GetInstance().GetDisplay()) {
            display->SetChatMessage("system", "画像を確認しています…");
        }
        return DescribeImageWithGrok(data_uri, description);
    });
    camera_explain_delegate_registered_ = true;
}

void GptRealtimeProtocol::UnregisterCameraExplainDelegate()
{
    if (!camera_explain_delegate_registered_) {
        return;
    }
    if (auto* camera = Board::GetInstance().GetCamera()) {
        camera->ClearExplainDelegate();
    }
    camera_explain_delegate_registered_ = false;
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
    std::string instructions = instructions_;
    // Per-tool usage (when to turn the head vs. take a photo, angle ranges, the OK/NG
    // confirmation) lives in each tool's own description, which is sent in the
    // function definitions -- no need to repeat it here. Keep only the behavioral
    // nudge that isn't expressible per-tool: don't refuse for lack of a camera.
    instructions +=
        "\n\nYou have a camera and head servos via the provided tools; never say you lack a camera.";
    cJSON_AddStringToObject(session, "instructions", instructions.c_str());

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

    AddStackChanFunctionTools(session);

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
    std::string instructions = instructions_;
    // Per-tool usage (when to turn the head vs. take a photo, angle ranges, the OK/NG
    // confirmation) lives in each tool's own description, which is sent in the
    // function definitions -- no need to repeat it here. Keep only the behavioral
    // nudge that isn't expressible per-tool: don't refuse for lack of a camera.
    instructions +=
        "\n\nYou have a camera and head servos via the provided tools; never say you lack a camera.";
    cJSON_AddStringToObject(session, "instructions", instructions.c_str());

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

    AddStackChanFunctionTools(session);

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
    bool function_active;
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        response_active = response_active_;
        function_active = function_active_;
        // Failsafe: never keep the mic muted forever if the post-photo response
        // never starts (e.g. a server hiccup) -- re-enable input after the cap.
        if (function_active && (GetHAL().millis() - function_active_since_ms_) > kFunctionMicMuteMaxMs) {
            function_active = false;
            function_active_ = false;
        }
    }
    // Suppress mic while a camera tool runs AND until its follow-up response starts
    // speaking: otherwise the resumed input stream makes Grok hold (never stream) the
    // post-photo answer, and server-VAD speech during the ~7s capture would open a
    // competing turn that collides with the result we inject.
    if (!IsAudioChannelOpened() || packet == nullptr || response_active || function_active) {
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
        function_active_ = false;
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
        function_active_ = false;
    }
    EmitTtsState("stop");
}

void GptRealtimeProtocol::SendWakeWordDetected(const std::string& wake_word)
{
    (void)wake_word;
}

void GptRealtimeProtocol::SendMcpMessage(const std::string& message)
{
    SendUserTextItem(message, true);
}

bool GptRealtimeProtocol::SendUserTextItem(const std::string& text, bool create_response)
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "conversation.item.create");
    cJSON* item = cJSON_AddObjectToObject(root, "item");
    cJSON_AddStringToObject(item, "type", "message");
    cJSON_AddStringToObject(item, "role", "user");
    cJSON* content = cJSON_AddArrayToObject(item, "content");
    cJSON* part = cJSON_CreateObject();
    cJSON_AddStringToObject(part, "type", "input_text");
    cJSON_AddStringToObject(part, "text", text.c_str());
    cJSON_AddItemToArray(content, part);
    char* json = cJSON_PrintUnformatted(root);
    bool ok = SendText(json);
    cJSON_free(json);
    cJSON_Delete(root);
    if (ok && create_response) {
        ok = SendText("{\"type\":\"response.create\"}");
    }
    return ok;
}

bool GptRealtimeProtocol::SupportsImageAttachment() const
{
    return true;
}

bool GptRealtimeProtocol::SendImageMessage(const std::string& data_uri, const std::string& text)
{
    if (provider_ == RealtimeProvider::Grok) {
        std::string description;
        if (!DescribeImageWithGrok(data_uri, description)) {
            return false;
        }

        std::string injected_text = text.empty() ? "The user approved and attached a camera photo." : text;
        injected_text += "\n\nVisual description from Grok image understanding:\n";
        injected_text += description;
        return SendUserTextItem(injected_text, false);
    }

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "conversation.item.create");
    cJSON* item = cJSON_AddObjectToObject(root, "item");
    cJSON_AddStringToObject(item, "type", "message");
    cJSON_AddStringToObject(item, "role", "user");
    cJSON* content = cJSON_AddArrayToObject(item, "content");

    if (!text.empty()) {
        cJSON* text_part = cJSON_CreateObject();
        cJSON_AddStringToObject(text_part, "type", "input_text");
        cJSON_AddStringToObject(text_part, "text", text.c_str());
        cJSON_AddItemToArray(content, text_part);
    }

    cJSON* image_part = cJSON_CreateObject();
    cJSON_AddStringToObject(image_part, "type", "input_image");
    cJSON_AddStringToObject(image_part, "image_url", data_uri.c_str());
    cJSON_AddStringToObject(image_part, "detail", "auto");
    cJSON_AddItemToArray(content, image_part);

    char* json = cJSON_PrintUnformatted(root);
    std::string message(json);
    cJSON_free(json);
    cJSON_Delete(root);
    return SendText(message);
}

void GptRealtimeProtocol::AddStackChanFunctionTools(cJSON* session)
{
    // Expose the MCP server's AI-visible tools as realtime function tools, so there
    // is a single source of tool definitions. Each MCP tool's name is sanitized
    // (dots -> underscores) for the wire; its inputSchema properties/required map
    // directly onto the function "parameters" object.
    cJSON* tools = cJSON_AddArrayToObject(session, "tools");

    int exposed = 0;
    for (McpTool* tool : McpServer::GetInstance().tools()) {
        if (tool->user_only()) {
            continue;  // user-only/system tools stay invisible to the model
        }

        cJSON* fn = cJSON_CreateObject();
        cJSON_AddStringToObject(fn, "type", "function");
        cJSON_AddStringToObject(fn, "name", SanitizeToolName(tool->name()).c_str());
        cJSON_AddStringToObject(fn, "description", tool->description().c_str());

        cJSON* params = cJSON_AddObjectToObject(fn, "parameters");
        cJSON_AddStringToObject(params, "type", "object");

        cJSON* props = cJSON_Parse(tool->properties().to_json().c_str());
        cJSON_AddItemToObject(params, "properties", props != nullptr ? props : cJSON_CreateObject());

        auto required = tool->properties().GetRequired();
        if (!required.empty()) {
            cJSON* required_array = cJSON_AddArrayToObject(params, "required");
            for (const auto& name : required) {
                cJSON_AddItemToArray(required_array, cJSON_CreateString(name.c_str()));
            }
        }

        cJSON_AddItemToArray(tools, fn);
        ++exposed;
    }

    ESP_LOGI(TAG, "Exposed %d MCP tools as realtime functions", exposed);
}

bool GptRealtimeProtocol::DescribeImageWithGrok(const std::string& data_uri, std::string& description)
{
    description.clear();
    if (api_key_.empty() || vision_model_.empty()) {
        LoadSettings();
    }
    if (api_key_.empty()) {
        SetError("Grok API key is not configured");
        return false;
    }

    // xAI image understanding via the OpenAI-compatible Chat Completions endpoint
    // (messages[].content[] with type "text" and type "image_url"). The Responses API
    // (/v1/responses with input_image) was accepted at the TLS layer but never returned
    // headers (60s timeout), so use the broadly-supported chat/completions shape.
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", vision_model_.c_str());
    cJSON_AddNumberToObject(root, "max_completion_tokens", vision_max_output_tokens_);
    // grok-4.3 defaults to "low" reasoning, which spends many uncapped reasoning tokens
    // before any visible output -> the request never returned headers in 60s. A photo
    // description needs no reasoning, so disable it for a fast response.
    cJSON_AddStringToObject(root, "reasoning_effort", "none");
    cJSON* messages = cJSON_AddArrayToObject(root, "messages");
    cJSON* message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "role", "user");
    cJSON* content = cJSON_AddArrayToObject(message, "content");

    cJSON* prompt = cJSON_CreateObject();
    cJSON_AddStringToObject(prompt, "type", "text");
    cJSON_AddStringToObject(prompt, "text",
                            "Describe this camera photo concisely and objectively for a voice agent. "
                            "Include visible objects, people, text, and spatial context. Do not answer the user yet.");
    cJSON_AddItemToArray(content, prompt);

    cJSON* image = cJSON_CreateObject();
    cJSON_AddStringToObject(image, "type", "image_url");
    cJSON* image_url = cJSON_AddObjectToObject(image, "image_url");
    cJSON_AddStringToObject(image_url, "url", data_uri.c_str());
    cJSON_AddStringToObject(image_url, "detail", vision_detail_.c_str());
    cJSON_AddItemToArray(content, image);

    cJSON_AddItemToArray(messages, message);

    char* json = cJSON_PrintUnformatted(root);
    std::string body(json);
    cJSON_free(json);
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Requesting Grok vision: model=%s detail=%s body=%u bytes (data_uri=%u)", vision_model_.c_str(),
             vision_detail_.c_str(), (unsigned)body.size(), (unsigned)data_uri.size());

    // Use ESP-IDF's esp_http_client (blocking I/O on this task) instead of the board's
    // custom HttpClient: the latter's TLS POST to api.x.ai never received a response
    // (its low-priority receive task is starved while realtime audio runs), so the
    // request always hit the 60s timeout even though the endpoint replies in <1s.
    std::string response;
    int status_code = 0;
    {
        esp_http_client_config_t config = {};
        config.url = "https://api.x.ai/v1/chat/completions";
        config.method = HTTP_METHOD_POST;
        config.timeout_ms = 60000;
        config.crt_bundle_attach = esp_crt_bundle_attach;
        config.buffer_size = 1536;
        config.buffer_size_tx = 1536;
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client == nullptr) {
            ESP_LOGE(TAG, "Failed to init Grok vision HTTP client");
            return false;
        }
        std::string auth = "Bearer " + api_key_;
        esp_http_client_set_header(client, "Authorization", auth.c_str());
        esp_http_client_set_header(client, "Content-Type", "application/json");

        esp_err_t err = esp_http_client_open(client, body.size());
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Grok vision open failed: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            return false;
        }
        size_t sent = 0;
        bool write_ok = true;
        while (sent < body.size()) {
            int wlen = esp_http_client_write(client, body.data() + sent, body.size() - sent);
            if (wlen <= 0) {
                write_ok = false;
                break;
            }
            sent += wlen;
        }
        if (!write_ok) {
            ESP_LOGE(TAG, "Grok vision write failed after %u/%u bytes", (unsigned)sent, (unsigned)body.size());
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return false;
        }
        esp_http_client_fetch_headers(client);
        status_code = esp_http_client_get_status_code(client);
        char buf[512];
        int rd;
        while ((rd = esp_http_client_read(client, buf, sizeof(buf))) > 0) {
            response.append(buf, rd);
        }
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
    }

    ESP_LOGI(TAG, "Grok vision response: status=%d len=%u body=%.400s", status_code, (unsigned)response.size(),
             response.c_str());
    if (status_code < 200 || status_code >= 300) {
        ESP_LOGE(TAG, "Grok image understanding failed, status=%d, body=%s", status_code, response.c_str());
        return false;
    }

    cJSON* response_json = cJSON_Parse(response.c_str());
    if (response_json == nullptr) {
        ESP_LOGE(TAG, "Failed to parse Grok image understanding response");
        return false;
    }
    // Chat Completions: choices[0].message.content (string).
    cJSON* choices = cJSON_GetObjectItem(response_json, "choices");
    if (cJSON_IsArray(choices)) {
        cJSON* first = cJSON_GetArrayItem(choices, 0);
        cJSON* msg = first ? cJSON_GetObjectItem(first, "message") : nullptr;
        cJSON* msg_content = msg ? cJSON_GetObjectItem(msg, "content") : nullptr;
        if (cJSON_IsString(msg_content)) {
            description = msg_content->valuestring;
        }
    }
    if (description.empty()) {
        description = ExtractTextFromJson(response_json);
    }
    cJSON_Delete(response_json);
    if (description.empty()) {
        ESP_LOGE(TAG, "Grok image understanding response had no text");
        return false;
    }

    ESP_LOGI(TAG, "Grok image description: %s", description.c_str());
    return true;
}

std::string GptRealtimeProtocol::ExtractTextFromJson(const cJSON* root) const
{
    if (root == nullptr) {
        return {};
    }
    if (cJSON_IsObject(root)) {
        auto type = cJSON_GetObjectItem(root, "type");
        auto text = cJSON_GetObjectItem(root, "text");
        if (cJSON_IsString(text) &&
            (!cJSON_IsString(type) || strcmp(type->valuestring, "output_text") == 0 ||
             strcmp(type->valuestring, "text") == 0)) {
            return text->valuestring;
        }
        auto output_text = cJSON_GetObjectItem(root, "output_text");
        if (cJSON_IsString(output_text)) {
            return output_text->valuestring;
        }
        for (auto child = root->child; child != nullptr; child = child->next) {
            auto found = ExtractTextFromJson(child);
            if (!found.empty()) {
                return found;
            }
        }
    } else if (cJSON_IsArray(root)) {
        for (auto child = root->child; child != nullptr; child = child->next) {
            auto found = ExtractTextFromJson(child);
            if (!found.empty()) {
                return found;
            }
        }
    }
    return {};
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
        if (strstr(type->valuestring, ".delta") == nullptr) {
            ESP_LOGI(TAG, "Realtime event: %s", type->valuestring);  // diag: see post-function-output flow
        }
        if (strcmp(type->valuestring, "session.created") == 0 || strcmp(type->valuestring, "session.updated") == 0) {
            xEventGroupSetBits(event_group_handle_, GPT_REALTIME_SESSION_READY_EVENT);
        } else if (strcmp(type->valuestring, "response.output_audio.delta") == 0 ||
                   strcmp(type->valuestring, "response.audio.delta") == 0) {
            HandleAudioDelta(root);
        } else if (strcmp(type->valuestring, "response.output_item.added") == 0 ||
                   strcmp(type->valuestring, "response.output_item.done") == 0) {
            RememberFunctionCallName(root);
        } else if (strcmp(type->valuestring, "response.function_call_arguments.done") == 0) {
            HandleFunctionCall(root);
        } else if (strcmp(type->valuestring, "response.output_audio.done") == 0 ||
                   strcmp(type->valuestring, "response.audio.done") == 0 ||
                   strcmp(type->valuestring, "response.done") == 0) {
            if (strcmp(type->valuestring, "response.done") == 0 && HandleFunctionCallsFromResponseDone(root)) {
                cJSON_Delete(root);
                return;
            }
            // The server finishes sending faster than real time. Don't stop now or
            // we'd cut playback short; mark the response complete and let the output
            // audio task emit "stop" once it has played out the buffered audio.
            // (A text-only response has no audio to drain, so end it immediately.)
            bool text_only_stop = false;
            std::string final_text;
            {
                std::lock_guard<std::mutex> lock(output_audio_mutex_);
                final_text = response_transcript_;  // full text (display updates are throttled)
                // A response finished (with or without audio): release mic suppression.
                function_active_ = false;
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

void GptRealtimeProtocol::RememberFunctionCallName(const cJSON* root)
{
    auto item = cJSON_GetObjectItem(root, "item");
    if (!cJSON_IsObject(item)) {
        return;
    }
    auto type = cJSON_GetObjectItem(item, "type");
    auto name = cJSON_GetObjectItem(item, "name");
    auto call_id = cJSON_GetObjectItem(item, "call_id");
    if (cJSON_IsString(type) && strcmp(type->valuestring, "function_call") == 0 && cJSON_IsString(name) &&
        cJSON_IsString(call_id)) {
        std::lock_guard<std::mutex> lock(function_call_mutex_);
        function_call_names_[call_id->valuestring] = name->valuestring;
        ESP_LOGI(TAG, "Remember function call: call_id=%s name=%s", call_id->valuestring, name->valuestring);
    }
}

void GptRealtimeProtocol::HandleFunctionCall(const cJSON* root)
{
    auto name = cJSON_GetObjectItem(root, "name");
    auto call_id = cJSON_GetObjectItem(root, "call_id");
    auto arguments = cJSON_GetObjectItem(root, "arguments");
    if (!cJSON_IsString(call_id)) {
        ESP_LOGW(TAG, "Function call event missing call_id");
        return;
    }
    std::string function_name;
    if (cJSON_IsString(name)) {
        function_name = name->valuestring;
    } else {
        std::lock_guard<std::mutex> lock(function_call_mutex_);
        auto remembered = function_call_names_.find(call_id->valuestring);
        if (remembered != function_call_names_.end()) {
            function_name = remembered->second;
        }
    }
    if (function_name.empty()) {
        ESP_LOGW(TAG, "Function call event missing name for call_id=%s", call_id->valuestring);
        return;
    }

    DispatchFunctionCall(call_id->valuestring, function_name,
                         cJSON_IsString(arguments) ? arguments->valuestring : "{}");
}

void GptRealtimeProtocol::DispatchFunctionCall(const std::string& call_id, const std::string& function_name,
                                               const std::string& arguments_json)
{
    {
        std::lock_guard<std::mutex> lock(function_call_mutex_);
        // Grok emits both response.function_call_arguments.done and a function_call
        // entry inside response.done for the same call_id. Run each call only once.
        if (!dispatched_call_ids_.insert(call_id).second) {
            ESP_LOGI(TAG, "Skip duplicate function call: call_id=%s", call_id.c_str());
            return;
        }
    }

    // Stop feeding mic audio immediately and keep it muted through the tool run and
    // the follow-up response, so server-VAD can't open a competing turn and the input
    // stream doesn't make Grok stall the post-photo answer (see function_active_).
    {
        std::lock_guard<std::mutex> lock(output_audio_mutex_);
        function_active_ = true;
        function_active_since_ms_ = GetHAL().millis();
    }

    // The camera tool blocks on a touch OK/NG confirmation and a synchronous Grok
    // vision HTTP request. Running it inline would freeze the WebSocket receive task
    // (no more audio/events processed -> "voice stops after a photo"). Spawning a
    // dedicated task here fails under memory pressure ("pthread: Failed to create
    // task!" -> abort), so run it on the main application loop instead -- the same
    // proven pattern as the MCP camera tool. It has a large stack, leaves the receive
    // task free, and SendText is mutex-protected so emitting from there is safe.
    Application::GetInstance().Schedule([this, call_id, function_name, arguments_json]() {
        cJSON* args = cJSON_Parse(arguments_json.c_str());
        if (args == nullptr) {
            args = cJSON_CreateObject();
        }
        // Route every function call through the MCP server so tool definitions and
        // execution share one implementation. The wire name is the dot->underscore
        // sanitized MCP name; resolve it back, then invoke. Camera tools run their
        // MCP capture/confirm flow and produce a Grok vision description via the
        // explain delegate registered in OpenAudioChannel().
        std::string mcp_name = ResolveMcpToolName(function_name);
        bool is_camera = IsCameraToolName(mcp_name);
        std::string output;
        bool ok = false;
        try {
            ESP_LOGI(TAG, "Execute function call: call_id=%s name=%s mcp=%s args=%s", call_id.c_str(),
                     function_name.c_str(), mcp_name.c_str(), arguments_json.c_str());
            if (mcp_name.empty()) {
                throw std::runtime_error("Unknown function: " + function_name);
            }
            output = McpServer::GetInstance().CallToolSync(mcp_name, args);
            ok = true;
        } catch (const std::exception& e) {
            ok = false;
            cJSON* err = cJSON_CreateObject();
            cJSON_AddBoolToObject(err, "success", false);
            cJSON_AddStringToObject(err, "error", e.what());
            char* err_json = cJSON_PrintUnformatted(err);
            output.assign(err_json);
            cJSON_free(err_json);
            cJSON_Delete(err);
        }
        cJSON_Delete(args);

        // Drop any mic audio server-VAD buffered just before we suppressed input,
        // so it can't fire a stray turn that races our function_call_output below.
        SendText("{\"type\":\"input_audio_buffer.clear\"}");

        cJSON* response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "type", "conversation.item.create");
        cJSON* item = cJSON_AddObjectToObject(response, "item");
        cJSON_AddStringToObject(item, "type", "function_call_output");
        cJSON_AddStringToObject(item, "call_id", call_id.c_str());
        cJSON_AddStringToObject(item, "output", output.c_str());
        char* json = cJSON_PrintUnformatted(response);
        bool out_ok = SendText(json);
        cJSON_free(json);
        cJSON_Delete(response);
        // Auto-speak is NOT possible from the client here: Grok's Voice Agent does not
        // stream audio for a client-issued response.create. Verified on device across
        // four approaches (bare response.create, one with instructions, an injected user
        // turn, and an explicit output_modalities:["audio"]) -- each opened a response,
        // added a content part, then went silent (no audio delta, no response.done) until
        // the user spoke. Only server-VAD (real user speech) turns produce audio. So we
        // intentionally do NOT send response.create. The visual description is now in the
        // conversation via function_call_output, so the user's next spoken turn makes
        // Grok describe the photo from context. Prompt the user to ask.
        //
        // Only camera/photo tools get a follow-up hint + chime. Other tools (head
        // turn, LED, reminders, volume, ...) just let the model speak its reply -- a
        // "what did you see?" nudge would make no sense after e.g. a head turn.
        // Keep each line short: the avatar bubble caps at 2 lines and only auto-scrolls
        // overflow for streamed replies (onSpeechComplete), not one-shot system text.
        // - OK: the caption is ready but Grok won't speak it on its own (see above),
        //   so nudge the user to ask about the photo.
        // - NG: the user tapped reject, so there is no photo to ask about -- just
        //   acknowledge instead of telling them to ask "what did you see?".
        if (is_camera) {
            auto display = Board::GetInstance().GetDisplay();
            if (display != nullptr) {
                if (ok) {
                    display->SetChatMessage("system", "「何が見えた？」と聞いてね");
                } else if (output.find("rejected") != std::string::npos) {
                    display->SetChatMessage("system", "わかった、やめとくね");
                } else {
                    display->SetChatMessage("system", "うまく撮れなかったみたい");
                }
            }
            hal_bridge::app_play_sound(OGG_NEW_NOTIFICATION);
        }
        ESP_LOGI(TAG, "Function output sent: call_id=%s output_ok=%d camera=%d output=%.160s",
                 call_id.c_str(), out_ok, static_cast<int>(is_camera), output.c_str());

        {
            std::lock_guard<std::mutex> lock(function_call_mutex_);
            function_call_names_.erase(call_id);
            // Keep dispatched_call_ids_ so a late response.done duplicate stays suppressed.
        }
        // Resume the mic now so the user can immediately ask about the photo.
        {
            std::lock_guard<std::mutex> lock(output_audio_mutex_);
            function_active_ = false;
        }
    });
}

bool GptRealtimeProtocol::HandleFunctionCallsFromResponseDone(const cJSON* root)
{
    auto response = cJSON_GetObjectItem(root, "response");
    auto items = cJSON_GetObjectItem(response, "output");
    if (!cJSON_IsArray(items)) {
        return false;
    }

    bool handled = false;
    cJSON* item = nullptr;
    cJSON_ArrayForEach(item, items) {
        auto type = cJSON_GetObjectItem(item, "type");
        if (!cJSON_IsString(type) || strcmp(type->valuestring, "function_call") != 0) {
            continue;
        }
        auto name = cJSON_GetObjectItem(item, "name");
        auto call_id = cJSON_GetObjectItem(item, "call_id");
        auto arguments = cJSON_GetObjectItem(item, "arguments");
        if (!cJSON_IsString(name) || !cJSON_IsString(call_id)) {
            continue;
        }

        // DispatchFunctionCall dedupes by call_id (this is usually the duplicate of the
        // response.function_call_arguments.done event) and runs off the receive task.
        DispatchFunctionCall(call_id->valuestring, name->valuestring,
                             cJSON_IsString(arguments) ? arguments->valuestring : "{}");
        handled = true;
    }

    // Return true when the response carried function calls so the caller skips the
    // normal terminal handling; the response.create is sent by the worker.
    return handled;
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
        // The post-photo response is now speaking; mic suppression did its job.
        function_active_ = false;
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
