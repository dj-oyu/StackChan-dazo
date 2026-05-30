#include "audio_service.h"
#include <esp_log.h>
#include <cstring>
#include <cmath>
#include <cstdint>
#include "esp_dsp.h"
#include <hal/board/hal_bridge.h>

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

#if CONFIG_USE_AUDIO_PROCESSOR
#include "processors/afe_audio_processor.h"
#else
#include "processors/no_audio_processor.h"
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#include "wake_words/afe_wake_word.h"
#include "wake_words/custom_wake_word.h"
#else
#include "wake_words/esp_wake_word.h"
#endif

#define TAG "AudioService"

AudioService::AudioService() {
    event_group_ = xEventGroupCreate();
}

AudioService::~AudioService() {
    if (event_group_ != nullptr) {
        vEventGroupDelete(event_group_);
    }
    if (opus_encoder_ != nullptr) {
        esp_opus_enc_close(opus_encoder_);
    }
    if (opus_decoder_ != nullptr) {
        esp_opus_dec_close(opus_decoder_);
    }
    if (input_resampler_ != nullptr) {
        esp_ae_rate_cvt_close(input_resampler_);
    }
    if (output_resampler_ != nullptr) {
        esp_ae_rate_cvt_close(output_resampler_);
    }
}

void AudioService::Initialize(AudioCodec* codec) {
    // During speaking we pause the AFE fetch side, so its mic FEED ringbuffer
    // overflows and esp-sr logs "Ringbuffer of AFE(FEED) is full" dozens of times
    // a second. That mic data is unused while speaking, but the warning flood
    // blocks the audio tasks on UART (~8ms/line at 115200) and audibly chops
    // playback. Silence AFE warnings; real failures still surface at ERROR.
    esp_log_level_set("AFE", ESP_LOG_ERROR);

    codec_ = codec;
    codec_->Start();

    esp_opus_dec_cfg_t opus_dec_cfg = OPUS_DEC_CFG(codec->output_sample_rate(), OPUS_FRAME_DURATION_MS);
    auto ret = esp_opus_dec_open(&opus_dec_cfg, sizeof(esp_opus_dec_cfg_t), &opus_decoder_);
    if (opus_decoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio decoder, error code: %d", ret);
    } else {
        decoder_sample_rate_ = codec->output_sample_rate();
        decoder_duration_ms_ = OPUS_FRAME_DURATION_MS;
        decoder_frame_size_ = decoder_sample_rate_ / 1000 * OPUS_FRAME_DURATION_MS;
    }
    esp_opus_enc_config_t opus_enc_cfg = AS_OPUS_ENC_CONFIG();
    ret = esp_opus_enc_open(&opus_enc_cfg, sizeof(esp_opus_enc_config_t), &opus_encoder_);
    if (opus_encoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio encoder, error code: %d", ret);
    } else {
        encoder_sample_rate_ = 16000;
        encoder_duration_ms_ = OPUS_FRAME_DURATION_MS;
        esp_opus_enc_get_frame_size(opus_encoder_, &encoder_frame_size_, &encoder_outbuf_size_);
        encoder_frame_size_ = encoder_frame_size_ / sizeof(int16_t);
    }

    if (codec->input_sample_rate() != 16000) {
        esp_ae_rate_cvt_cfg_t input_resampler_cfg = RATE_CVT_CFG(
            codec->input_sample_rate(), ESP_AUDIO_SAMPLE_RATE_16K, codec->input_channels());
        auto resampler_ret = esp_ae_rate_cvt_open(&input_resampler_cfg, &input_resampler_);
        if (input_resampler_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create input resampler, error code: %d", resampler_ret);
        }
    }

#if CONFIG_USE_AUDIO_PROCESSOR
    audio_processor_ = std::make_unique<AfeAudioProcessor>();
#else
    audio_processor_ = std::make_unique<NoAudioProcessor>();
#endif

    audio_processor_->OnOutput([this](std::vector<int16_t>&& data) {
        if (send_raw_pcm_) {
            // Forward the captured PCM straight to the send queue (no Opus encode)
            // for protocols that want PCM. Drop if the queue is full (backpressure).
            auto packet = std::make_unique<AudioStreamPacket>();
            packet->format = AudioStreamFormat::kPcm;
            packet->sample_rate = 16000;
            packet->frame_duration = OPUS_FRAME_DURATION_MS;
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data.data());
            packet->payload.assign(bytes, bytes + data.size() * sizeof(int16_t));
            {
                std::lock_guard<std::mutex> lock(audio_queue_mutex_);
                if (audio_send_queue_.size() < MAX_SEND_PACKETS_IN_QUEUE) {
                    audio_send_queue_.push_back(std::move(packet));
                }
            }
            if (callbacks_.on_send_queue_available) {
                callbacks_.on_send_queue_available();
            }
            return;
        }
        PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue, std::move(data));
    });

    audio_processor_->OnVadStateChange([this](bool speaking) {
        voice_detected_ = speaking;
        if (callbacks_.on_vad_change) {
            callbacks_.on_vad_change(speaking);
        }
    });

    esp_timer_create_args_t audio_power_timer_args = {
        .callback = [](void* arg) {
            AudioService* audio_service = (AudioService*)arg;
            audio_service->CheckAndUpdateAudioPowerState();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "audio_power_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&audio_power_timer_args, &audio_power_timer_);
}

void AudioService::Start() {
    service_stopped_ = false;
    xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING | AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING);

    esp_timer_start_periodic(audio_power_timer_, 1000000);

#if CONFIG_USE_AUDIO_PROCESSOR
    /* Start the audio input task */
    xTaskCreatePinnedToCore([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioInputTask();
        vTaskDelete(NULL);
    }, "audio_input", 2048 * 3, this, 8, &audio_input_task_handle_, 0);

    /* Start the audio output task */
    xTaskCreate([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioOutputTask();
        vTaskDelete(NULL);
    }, "audio_output", 2048 * 2, this, 4, &audio_output_task_handle_);
#else
    /* Start the audio input task */
    xTaskCreate([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioInputTask();
        vTaskDelete(NULL);
    }, "audio_input", 2048 * 2, this, 8, &audio_input_task_handle_);

    /* Start the audio output task */
    xTaskCreate([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioOutputTask();
        vTaskDelete(NULL);
    }, "audio_output", 2048, this, 4, &audio_output_task_handle_);
#endif

    /* Start the opus codec task */
    xTaskCreate([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->OpusCodecTask();
        vTaskDelete(NULL);
    }, "opus_codec", 2048 * 12, this, 2, &opus_codec_task_handle_);
}

void AudioService::Stop() {
    esp_timer_stop(audio_power_timer_);
    service_stopped_ = true;
    xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING |
        AS_EVENT_WAKE_WORD_RUNNING |
        AS_EVENT_AUDIO_PROCESSOR_RUNNING);

    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    audio_encode_queue_.clear();
    audio_decode_queue_.clear();
    audio_playback_queue_.clear();
    audio_testing_queue_.clear();
    audio_queue_cv_.notify_all();
}

bool AudioService::ReadAudioData(std::vector<int16_t>& data, int sample_rate, int samples) {
    if (!codec_->input_enabled()) {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableInput(true);
    }

    if (codec_->input_sample_rate() != sample_rate) {
        data.resize(samples * codec_->input_sample_rate() / sample_rate * codec_->input_channels());
        if (!codec_->InputData(data)) {
            return false;
        }
        if (input_resampler_ != nullptr) {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            uint32_t in_sample_num = data.size() / codec_->input_channels();
            uint32_t output_samples = 0;
            esp_ae_rate_cvt_get_max_out_sample_num(input_resampler_, in_sample_num, &output_samples);
            auto resampled = std::vector<int16_t>(output_samples * codec_->input_channels());
            uint32_t actual_output = output_samples;
            esp_ae_rate_cvt_process(input_resampler_, (esp_ae_sample_t)data.data(), in_sample_num,
                                   (esp_ae_sample_t)resampled.data(), &actual_output);
            resampled.resize(actual_output * codec_->input_channels());
            data = std::move(resampled);
        }
    } else {
        data.resize(samples * codec_->input_channels());
        if (!codec_->InputData(data)) {
            return false;
        }
    }

    /* Update the last input time */
    last_input_time_ = std::chrono::steady_clock::now();
    debug_statistics_.input_count++;

#if CONFIG_USE_AUDIO_DEBUGGER
    // 音频调试：发送原始音频数据
    if (audio_debugger_ == nullptr) {
        audio_debugger_ = std::make_unique<AudioDebugger>();
    }
    audio_debugger_->Feed(data);
#endif

    return true;
}

void AudioService::AudioInputTask() {
    std::vector<int16_t> input_frame;
    input_frame.reserve(160 * codec_->input_channels());

    while (true) {
        EventBits_t bits = xEventGroupWaitBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING |
            AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING,
            pdFALSE, pdFALSE, portMAX_DELAY);

        if (service_stopped_) {
            break;
        }
        if (audio_input_need_warmup_) {
            audio_input_need_warmup_ = false;
            vTaskDelay(pdMS_TO_TICKS(120));
            continue;
        }

        /* Used for audio testing in NetworkConfiguring mode by clicking the BOOT button */
        if (bits & AS_EVENT_AUDIO_TESTING_RUNNING) {
            if (audio_testing_queue_.size() >= AUDIO_TESTING_MAX_DURATION_MS / OPUS_FRAME_DURATION_MS) {
                ESP_LOGW(TAG, "Audio testing queue is full, stopping audio testing");
                EnableAudioTesting(false);
                continue;
            }
            std::vector<int16_t> data;
            int samples = OPUS_FRAME_DURATION_MS * 16000 / 1000;
            if (ReadAudioData(data, 16000, samples)) {
                // If input channels is 2, we need to fetch the left channel data
                if (codec_->input_channels() == 2) {
                    size_t mono_size = data.size() / 2;
                    for (size_t i = 0, j = 0; i < mono_size; ++i, j += 2) {
                        data[i] = data[j];
                    }
                    data.resize(mono_size);
                }
                PushTaskToEncodeQueue(kAudioTaskTypeEncodeToTestingQueue, std::move(data));
                continue;
            }
        }

        /* Feed the wake word and/or audio processor */
        if (bits & (AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING)) {
            int samples = 160; // 10ms
            if (ReadAudioData(input_frame, 16000, samples)) {
                if (bits & AS_EVENT_WAKE_WORD_RUNNING) {
                    wake_word_->Feed(input_frame);
                    MaybeEmitSpectrum(input_frame);  // [TEMP DIAG] idle music survey
                }
                if (bits & AS_EVENT_AUDIO_PROCESSOR_RUNNING) {
                    audio_processor_->Feed(std::move(input_frame));
                }
                continue;
            }
        }

        // Read timeout/error should not terminate the input task.
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "Audio input task stopped");
}

void AudioService::AudioOutputTask() {
    while (true) {
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);
        audio_queue_cv_.wait(lock, [this]() { return !audio_playback_queue_.empty() || service_stopped_; });
        if (service_stopped_) {
            break;
        }

        auto task = std::move(audio_playback_queue_.front());
        audio_playback_queue_.pop_front();
        audio_queue_cv_.notify_all();
        lock.unlock();

        if (!codec_->output_enabled()) {
            esp_timer_stop(audio_power_timer_);
            esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
            codec_->EnableOutput(true);
        }

        codec_->OutputData(task->pcm);

        /* Update the last output time */
        last_output_time_ = std::chrono::steady_clock::now();
        debug_statistics_.playback_count++;

#if CONFIG_USE_SERVER_AEC
        /* Record the timestamp for server AEC */
        if (task->timestamp > 0) {
            lock.lock();
            timestamp_queue_.push_back(task->timestamp);
        }
#endif
    }

    ESP_LOGW(TAG, "Audio output task stopped");
}

void AudioService::OpusCodecTask() {
    while (true) {
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);
        audio_queue_cv_.wait(lock, [this]() {
            return service_stopped_ ||
                (!audio_encode_queue_.empty() && audio_send_queue_.size() < MAX_SEND_PACKETS_IN_QUEUE) ||
                (!audio_decode_queue_.empty() && audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE);
        });
        if (service_stopped_) {
            break;
        }

        /* Decode the audio from decode queue */
        if (!audio_decode_queue_.empty() && audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE) {
            auto packet = std::move(audio_decode_queue_.front());
            audio_decode_queue_.pop_front();
            audio_queue_cv_.notify_all();
            lock.unlock();

            auto task = std::make_unique<AudioTask>();
            task->type = kAudioTaskTypeDecodeToPlaybackQueue;
            task->timestamp = packet->timestamp;

            SetDecodeSampleRate(packet->sample_rate, packet->frame_duration);
            bool have_pcm = false;
            if (packet->format == AudioStreamFormat::kPcm) {
                // Already PCM (e.g. OpenAI realtime): skip the Opus decoder.
                const int16_t* samples = reinterpret_cast<const int16_t*>(packet->payload.data());
                size_t count = packet->payload.size() / sizeof(int16_t);
                task->pcm.assign(samples, samples + count);
                have_pcm = true;
            } else if (opus_decoder_ != nullptr) {
                task->pcm.resize(decoder_frame_size_);
                esp_audio_dec_in_raw_t raw = {
                    .buffer = (uint8_t *)(packet->payload.data()),
                    .len = (uint32_t)(packet->payload.size()),
                    .consumed = 0,
                    .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
                };
                esp_audio_dec_out_frame_t out_frame = {
                    .buffer = (uint8_t *)(task->pcm.data()),
                    .len = (uint32_t)(task->pcm.size() * sizeof(int16_t)),
                    .decoded_size = 0,
                };
                esp_audio_dec_info_t dec_info = {};
                std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
                auto ret = esp_opus_dec_decode(opus_decoder_, &raw, &out_frame, &dec_info);
                decoder_lock.unlock();
                if (ret == ESP_AUDIO_ERR_OK) {
                    task->pcm.resize(out_frame.decoded_size / sizeof(int16_t));
                    have_pcm = true;
                } else {
                    ESP_LOGE(TAG, "Failed to decode audio after resize, error code: %d", ret);
                }
            } else {
                ESP_LOGE(TAG, "Audio decoder is not configured");
            }

            if (have_pcm) {
                if (decoder_sample_rate_ != codec_->output_sample_rate() && output_resampler_ != nullptr) {
                    uint32_t target_size = 0;
                    esp_ae_rate_cvt_get_max_out_sample_num(output_resampler_, task->pcm.size(), &target_size);
                    std::vector<int16_t> resampled(target_size);
                    uint32_t actual_output = target_size;
                    esp_ae_rate_cvt_process(output_resampler_, (esp_ae_sample_t)task->pcm.data(), task->pcm.size(),
                                            (esp_ae_sample_t)resampled.data(), &actual_output);
                    resampled.resize(actual_output);
                    task->pcm = std::move(resampled);
                }
                lock.lock();
                audio_playback_queue_.push_back(std::move(task));
                audio_queue_cv_.notify_all();
            } else {
                lock.lock();
            }
            debug_statistics_.decode_count++;
        }
        /* Encode the audio to send queue */
        if (!audio_encode_queue_.empty() && audio_send_queue_.size() < MAX_SEND_PACKETS_IN_QUEUE) {
            auto task = std::move(audio_encode_queue_.front());
            audio_encode_queue_.pop_front();
            audio_queue_cv_.notify_all();
            lock.unlock();

            auto packet = std::make_unique<AudioStreamPacket>();
            packet->frame_duration = OPUS_FRAME_DURATION_MS;
            packet->sample_rate = 16000;
            packet->timestamp = task->timestamp;

            if (opus_encoder_ != nullptr && task->pcm.size() == encoder_frame_size_) {
                std::vector<uint8_t> buf(encoder_outbuf_size_);
                esp_audio_enc_in_frame_t in = {
                    .buffer = (uint8_t *)(task->pcm.data()),
                    .len = (uint32_t)(encoder_frame_size_ * sizeof(int16_t)),
                };
                esp_audio_enc_out_frame_t out = {
                    .buffer = buf.data(),
                    .len = (uint32_t)encoder_outbuf_size_,
                    .encoded_bytes = 0,
                };
                auto ret = esp_opus_enc_process(opus_encoder_, &in, &out);
                if (ret == ESP_AUDIO_ERR_OK) {
                    packet->payload.assign(buf.data(), buf.data() + out.encoded_bytes);

                    if (task->type == kAudioTaskTypeEncodeToSendQueue) {
                        {
                            std::lock_guard<std::mutex> lock2(audio_queue_mutex_);
                            audio_send_queue_.push_back(std::move(packet));
                        }
                        if (callbacks_.on_send_queue_available) {
                            callbacks_.on_send_queue_available();
                        }
                    } else if (task->type == kAudioTaskTypeEncodeToTestingQueue) {
                        std::lock_guard<std::mutex> lock2(audio_queue_mutex_);
                        audio_testing_queue_.push_back(std::move(packet));
                    }
                    debug_statistics_.encode_count++;
                } else {
                    ESP_LOGE(TAG, "Failed to encode audio, error code: %d", ret);
                }
            } else {
                ESP_LOGE(TAG, "Failed to encode audio: encoder not configured or invalid frame size (got %u, expected %u)",
                         task->pcm.size(), encoder_frame_size_);
            }
            lock.lock();
        }
    }

    ESP_LOGW(TAG, "Opus codec task stopped");
}

void AudioService::SetDecodeSampleRate(int sample_rate, int frame_duration) {
    if (decoder_sample_rate_ == sample_rate && decoder_duration_ms_ == frame_duration) {
        return;
    }
    std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
    if (opus_decoder_ != nullptr) {
        esp_opus_dec_close(opus_decoder_);
        opus_decoder_ = nullptr;
    }
    decoder_lock.unlock();
    esp_opus_dec_cfg_t opus_dec_cfg = OPUS_DEC_CFG(sample_rate, frame_duration);
    auto ret = esp_opus_dec_open(&opus_dec_cfg, sizeof(esp_opus_dec_cfg_t), &opus_decoder_);
    if (opus_decoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio decoder, error code: %d", ret);
        return;
    }
    decoder_sample_rate_ = sample_rate;
    decoder_duration_ms_ = frame_duration;
    decoder_frame_size_ = decoder_sample_rate_ / 1000 * frame_duration;

    auto codec = Board::GetInstance().GetAudioCodec();
    if (decoder_sample_rate_ != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", decoder_sample_rate_, codec->output_sample_rate());
        if (output_resampler_ != nullptr) {
            esp_ae_rate_cvt_close(output_resampler_);
            output_resampler_ = nullptr;
        }
        esp_ae_rate_cvt_cfg_t output_resampler_cfg = RATE_CVT_CFG(
            decoder_sample_rate_, codec->output_sample_rate(), ESP_AUDIO_MONO);
        auto resampler_ret = esp_ae_rate_cvt_open(&output_resampler_cfg, &output_resampler_);
        if (output_resampler_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create output resampler, error code: %d", resampler_ret);
        }
    }
}

void AudioService::PushTaskToEncodeQueue(AudioTaskType type, std::vector<int16_t>&& pcm) {
    auto task = std::make_unique<AudioTask>();
    task->type = type;
    task->pcm = std::move(pcm);
    /* Push the task to the encode queue */
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);

    /* If the task is to send queue, we need to set the timestamp */
    if (type == kAudioTaskTypeEncodeToSendQueue && !timestamp_queue_.empty()) {
        if (timestamp_queue_.size() <= MAX_TIMESTAMPS_IN_QUEUE) {
            task->timestamp = timestamp_queue_.front();
        } else {
            ESP_LOGW(TAG, "Timestamp queue (%u) is full, dropping timestamp", timestamp_queue_.size());
        }
        timestamp_queue_.pop_front();
    }

    audio_queue_cv_.wait(lock, [this]() { return audio_encode_queue_.size() < MAX_ENCODE_TASKS_IN_QUEUE; });
    audio_encode_queue_.push_back(std::move(task));
    audio_queue_cv_.notify_all();
}

bool AudioService::PushPacketToDecodeQueue(std::unique_ptr<AudioStreamPacket> packet, bool wait) {
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);
    if (audio_decode_queue_.size() >= MAX_DECODE_PACKETS_IN_QUEUE) {
        if (wait) {
            audio_queue_cv_.wait(lock, [this]() { return audio_decode_queue_.size() < MAX_DECODE_PACKETS_IN_QUEUE; });
        } else {
            return false;
        }
    }
    audio_decode_queue_.push_back(std::move(packet));
    audio_queue_cv_.notify_all();
    return true;
}

std::unique_ptr<AudioStreamPacket> AudioService::PopPacketFromSendQueue() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    if (audio_send_queue_.empty()) {
        return nullptr;
    }
    auto packet = std::move(audio_send_queue_.front());
    audio_send_queue_.pop_front();
    audio_queue_cv_.notify_all();
    return packet;
}

void AudioService::EncodeWakeWord() {
    if (wake_word_) {
        wake_word_->EncodeWakeWordData();
    }
}

const std::string& AudioService::GetLastWakeWord() const {
    return wake_word_->GetLastDetectedWakeWord();
}

std::unique_ptr<AudioStreamPacket> AudioService::PopWakeWordPacket() {
    auto packet = std::make_unique<AudioStreamPacket>();
    if (wake_word_->GetWakeWordOpus(packet->payload)) {
        return packet;
    }
    return nullptr;
}

void AudioService::EnableWakeWordDetection(bool enable) {
    if (!wake_word_) {
        return;
    }

    ESP_LOGD(TAG, "%s wake word detection", enable ? "Enabling" : "Disabling");
    if (enable) {
        if (!wake_word_initialized_) {
            if (!wake_word_->Initialize(codec_, models_list_)) {
                ESP_LOGE(TAG, "Failed to initialize wake word");
                return;
            }
            wake_word_initialized_ = true;
        }
        // Reset input resampler to clear cached data from previous mode (e.g. AudioProcessor)
        // This prevents buffer overflow when switching between different feed sizes
        {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            if (input_resampler_ != nullptr) {
                esp_ae_rate_cvt_reset(input_resampler_);
            }
        }
        wake_word_->Start();
        xEventGroupSetBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
    } else {
        wake_word_->Stop();
        xEventGroupClearBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
    }
}

void AudioService::EnableVoiceProcessing(bool enable) {
    ESP_LOGD(TAG, "%s voice processing", enable ? "Enabling" : "Disabling");
    if (enable) {
        if (!audio_processor_initialized_) {
            audio_processor_->Initialize(codec_, OPUS_FRAME_DURATION_MS, models_list_);
            audio_processor_initialized_ = true;
        }

        /* We should make sure no audio is playing */
        ResetDecoder();
        audio_input_need_warmup_ = true;
        // Reset input resampler to clear cached data from previous mode (e.g. WakeWord)
        // This prevents buffer overflow when switching between different feed sizes
        {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            if (input_resampler_ != nullptr) {
                esp_ae_rate_cvt_reset(input_resampler_);
            }
        }
        audio_processor_->Start();
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_PROCESSOR_RUNNING);
    } else {
        audio_processor_->Stop();
        xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_PROCESSOR_RUNNING);
    }
}

void AudioService::EnableAudioTesting(bool enable) {
    ESP_LOGI(TAG, "%s audio testing", enable ? "Enabling" : "Disabling");
    if (enable) {
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING);
    } else {
        xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING);
        /* Copy audio_testing_queue_ to audio_decode_queue_ */
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        audio_decode_queue_ = std::move(audio_testing_queue_);
        audio_queue_cv_.notify_all();
    }
}

void AudioService::EnableDeviceAec(bool enable) {
    ESP_LOGI(TAG, "%s device AEC", enable ? "Enabling" : "Disabling");
    if (!audio_processor_initialized_) {
        audio_processor_->Initialize(codec_, OPUS_FRAME_DURATION_MS, models_list_);
        audio_processor_initialized_ = true;
    }

    audio_processor_->EnableDeviceAec(enable);
}

void AudioService::SetCallbacks(AudioServiceCallbacks& callbacks) {
    callbacks_ = callbacks;
}

void AudioService::PlaySound(const std::string_view& ogg) {
    if (!codec_->output_enabled()) {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableOutput(true);
    }

    const auto* buf = reinterpret_cast<const uint8_t*>(ogg.data());
    size_t size = ogg.size();

    auto demuxer = std::make_unique<OggDemuxer>();
    demuxer->OnDemuxerFinished([this](const uint8_t* data, int sample_rate, size_t size){
        auto packet = std::make_unique<AudioStreamPacket>();
        packet->sample_rate = sample_rate;
        packet->frame_duration = 60;
        packet->payload.resize(size);
        std::memcpy(packet->payload.data(), data, size);
        PushPacketToDecodeQueue(std::move(packet), true);
    });
    demuxer->Reset();
    demuxer->Process(buf, size);
}

// [TEMP DIAG] Idle kick monitor: band-pass the mic to the kick band (~40-75 Hz) with a
// biquad and stream the per-frame energy ("[KICK]<m><hh>") at the mic frame rate
// (~100 Hz) so a PC tool / the on-device beat detector can run SHARP onset detection.
// A short per-frame energy keeps transients crisp (unlike a long FFT window, which
// smooths the envelope and erases the onset). Idle only; remove when done.
void AudioService::MaybeEmitSpectrum(const std::vector<int16_t>& frame) {
    if (spec_disabled_ || frame.empty()) {
        return;
    }
    if (!spec_inited_) {
        // Band-pass centered ~55 Hz, Q ~1.5 -> roughly 40-75 Hz.
        if (dsps_biquad_gen_bpf_f32(spec_kick_coeffs_, 55.0f / 16000.0f, 1.5f) != ESP_OK) {
            ESP_LOGW(TAG, "[KICK] biquad init failed; disabling kick monitor");
            spec_disabled_ = true;
            return;
        }
        spec_kick_w_[0] = spec_kick_w_[1] = 0.0f;
        spec_inited_ = true;
    }

    int nsm = (int)frame.size();
    float in[320], out[320];
    if (nsm > 320) nsm = 320;
    for (int i = 0; i < nsm; ++i) {
        in[i] = (float)frame[i] / 32768.0f;
    }
    dsps_biquad_f32(in, out, nsm, spec_kick_coeffs_, spec_kick_w_);

    float ms = 0.0f;
    for (int i = 0; i < nsm; ++i) {
        ms += out[i] * out[i];
    }
    ms /= (float)nsm;
    float db = 10.0f * log10f(ms + 1e-12f);
    int v = (int)((db + 90.0f) / 70.0f * 255.0f);  // map ~[-90,-20] dB -> 0..255
    if (v < 0) v = 0;
    if (v > 255) v = 255;

    // ([KICK] serial stream removed now that detection runs on-device; re-enable with
    //  printf("[KICK]%c%02x\n", mv?'1':'0', v); if the PC viewer is needed again.)
    BeatDetectStep(v, hal_bridge::servo_is_moving());
}

// Beat detector constants (tuned on the PC preview).
static constexpr int kBeatDecimate    = 3;          // 100 Hz -> ~33 Hz detection rate
static constexpr int kBeatHistN       = 24;
static constexpr int kBeatFluxN       = 3;          // local-mean baseline (~90 ms)
static constexpr float kBeatThreshK   = 1.0f;
static constexpr float kBeatThreshFloor = 2.5f;
static constexpr float kBeatLockSpread = 0.35f;  // lock readily; confidence dims wrong beats
static constexpr int64_t kBeatRefractoryUs = 180000;   // 180 ms -> <= ~330 BPM
static constexpr int64_t kBeatMaxIntervalUs = 2000000; // ignore gaps > 2 s as non-beats
static constexpr int64_t kBeatTimeoutUs = 2500000;     // unlock after 2.5 s with no kick
static constexpr int kBeatIntervalsN  = 8;

static void beat_isort(int64_t* a, int n) {
    for (int i = 1; i < n; ++i) {
        int64_t k = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; --j; }
        a[j + 1] = k;
    }
}

void AudioService::BeatDetectStep(int energy, bool moving) {
    // 1) Decimate the 100 Hz energy to ~33 Hz (smoother single peaks).
    beat_.dec_sum += (float)energy;
    if (++beat_.dec_cnt < kBeatDecimate) {
        return;
    }
    float sample = beat_.dec_sum / (float)beat_.dec_cnt;
    beat_.dec_sum = 0.0f;
    beat_.dec_cnt = 0;

    // 2) Flux vs a short local mean; adaptive noise from the history.
    float baseline = sample;
    if (beat_.hist_filled >= kBeatFluxN) {
        float s = 0.0f;
        for (int j = 0; j < kBeatFluxN; ++j) {
            int idx = (beat_.hist_pos - 1 - j + kBeatHistN) % kBeatHistN;
            s += beat_.hist[idx];
        }
        baseline = s / (float)kBeatFluxN;
    }
    float mean = 0.0f, var = 0.0f;
    if (beat_.hist_filled > 0) {
        for (int j = 0; j < beat_.hist_filled; ++j) mean += beat_.hist[j];
        mean /= (float)beat_.hist_filled;
        for (int j = 0; j < beat_.hist_filled; ++j) { float d = beat_.hist[j] - mean; var += d * d; }
        var /= (float)beat_.hist_filled;
    }
    float noise = sqrtf(var);
    float flux  = sample - baseline;

    beat_.hist[beat_.hist_pos] = sample;
    beat_.hist_pos = (beat_.hist_pos + 1) % kBeatHistN;
    if (beat_.hist_filled < kBeatHistN) beat_.hist_filled++;

    int64_t now = esp_timer_get_time();

    // 3) Unlock if the music stopped (no kick for a while).
    if (beat_.locked && beat_.last_onset_us != 0 && (now - beat_.last_onset_us) > kBeatTimeoutUs) {
        beat_.locked = false;
        beat_.intervals_cnt = 0;
        hal_bridge::beat_publish(0, 0, false, 0.0f);
        ESP_LOGI(TAG, "[BEAT] UNLOCK");
    }

    if (moving) {
        return;  // servo noise: keep the baseline fresh but don't fire onsets
    }

    // 4) Onset?
    float thr = (kBeatThreshK * noise > kBeatThreshFloor) ? (kBeatThreshK * noise) : kBeatThreshFloor;
    if (flux <= thr || (now - beat_.last_onset_us) <= kBeatRefractoryUs) {
        return;
    }

    int64_t prev = beat_.last_onset_us;
    beat_.last_onset_us = now;
    if (prev == 0) {
        return;  // first onset: no interval yet
    }
    int64_t interval = now - prev;
    if (interval <= 0 || interval > kBeatMaxIntervalUs) {
        return;
    }

    // 5) Track tempo: octave-fold recent intervals toward their median, take the median.
    beat_.intervals[beat_.intervals_pos] = interval;
    beat_.intervals_pos = (beat_.intervals_pos + 1) % kBeatIntervalsN;
    if (beat_.intervals_cnt < kBeatIntervalsN) beat_.intervals_cnt++;
    if (beat_.intervals_cnt < 4) {
        return;
    }
    int m = beat_.intervals_cnt;
    int64_t s1[kBeatIntervalsN];
    for (int j = 0; j < m; ++j) s1[j] = beat_.intervals[j];
    beat_isort(s1, m);
    int64_t ref = s1[m / 2];
    int64_t folded[kBeatIntervalsN];
    for (int j = 0; j < m; ++j) {
        int64_t b = beat_.intervals[j];
        while (b < ref * 2 / 3) b *= 2;
        while (b > ref * 3 / 2) b /= 2;
        folded[j] = b;
    }
    beat_isort(folded, m);
    int64_t period = folded[m / 2];
    if (period <= 0) {
        return;
    }
    float spread = (float)(folded[m - 1] - folded[0]) / (float)period;
    // Confidence 0..1 from interval consistency: tight intervals -> 1 (bright/correct),
    // loose -> 0 (dim, so a wrong beat is barely noticeable).
    float conf = 1.0f - spread / kBeatLockSpread;
    if (conf < 0.0f) conf = 0.0f;
    if (conf > 1.0f) conf = 1.0f;

    // 6) Lock / PLL the phase, then publish for the LED task.
    if (!beat_.locked) {
        if (spread < kBeatLockSpread) {
            beat_.locked    = true;
            beat_.period_us = period;
            beat_.anchor_us = now;
            hal_bridge::beat_publish(beat_.anchor_us, beat_.period_us, true, conf);
            ESP_LOGI(TAG, "[BEAT] LOCK ~%d bpm conf=%.2f", (int)(60000000LL / period), conf);
        }
        return;
    }
    beat_.period_us = period;
    // Nudge the phase anchor toward this onset (gentle PLL) so it stays aligned
    // without resetting every beat (the LED task derives beats from anchor+k*period).
    int64_t kk = (now - beat_.anchor_us + beat_.period_us / 2) / beat_.period_us;
    int64_t expected = beat_.anchor_us + kk * beat_.period_us;
    beat_.anchor_us += (now - expected) / 8;
    hal_bridge::beat_publish(beat_.anchor_us, beat_.period_us, true, conf);
}

void AudioService::PlayPcm(const int16_t* samples, size_t count, int sample_rate) {
    if (samples == nullptr || count == 0) {
        return;
    }
    if (!codec_->output_enabled()) {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableOutput(true);
    }
    auto packet = std::make_unique<AudioStreamPacket>();
    packet->sample_rate = sample_rate;
    packet->frame_duration = 60;
    packet->format = AudioStreamFormat::kPcm;
    packet->payload.resize(count * sizeof(int16_t));
    std::memcpy(packet->payload.data(), samples, packet->payload.size());
    // Don't block the caller (often a state-change handler); drop if the queue is full.
    PushPacketToDecodeQueue(std::move(packet), false);
}

// Synthesize a walkie-talkie squelch "ザザッ/サーッ": high-pass-filtered white noise
// shaped as a BURST (quick attack, flat sustain, quick release) so it reads as static,
// not a percussive clap. esp-dsp's high-pass biquad gives the bright hiss; an optional
// amplitude modulation adds the buzzy "ザザ" rasp. Output is normalized to a fixed peak
// so loudness is predictable regardless of the filter's gain.
void AudioService::SynthSquelchCue(std::vector<int16_t>& out, int sample_rate, int duration_ms, float amplitude,
                                   bool rasp) {
    int n = (int)((int64_t)sample_rate * duration_ms / 1000);
    if (n <= 0) {
        out.clear();
        return;
    }
    std::vector<float> noise(n);
    std::vector<float> filtered(n);

    // White noise via xorshift32 (fixed seed -> consistent cue), mapped to [-1, 1).
    uint32_t s = 0x1234567u;
    for (int i = 0; i < n; ++i) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        noise[i] = ((float)(s & 0x00FFFFFFu) / (float)0x00800000u) - 1.0f;
    }

    // High-pass the noise -> bright "シャー" hiss (squelch noise is high-frequency).
    // Two cascaded stages make the cutoff steeper / the timbre brighter.
    float coeffs[5];
    float w1[2] = {0.0f, 0.0f};
    float w2[2] = {0.0f, 0.0f};
    float fc = 2000.0f / (float)sample_rate;  // normalized cutoff
    dsps_biquad_gen_hpf_f32(coeffs, fc, 0.707f);
    dsps_biquad_f32(noise.data(), filtered.data(), n, coeffs, w1);
    dsps_biquad_f32(filtered.data(), filtered.data(), n, coeffs, w2);

    // Attack/Sustain/Release envelope: a flat-topped burst, NOT a front-loaded decay.
    // The sustain is what makes it a "static burst" instead of a clap.
    int attack  = sample_rate * 8 / 1000;
    int release = sample_rate * 45 / 1000;
    if (attack < 1) attack = 1;
    if (release < 1) release = 1;
    if (attack + release > n) {
        attack  = n / 4;
        release = n / 4;
    }
    int sustain_end = n - release;
    for (int i = 0; i < n; ++i) {
        float env;
        if (i < attack) {
            env = (float)i / (float)attack;
        } else if (i < sustain_end) {
            env = 1.0f;
        } else {
            env = (float)(n - i) / (float)release;
        }
        filtered[i] *= env;
    }

    // Optional amplitude modulation -> buzzy "ザザ" rasp (smooth hiss when off).
    if (rasp) {
        float wam = 2.0f * (float)M_PI * 90.0f / (float)sample_rate;  // ~90 Hz buzz
        for (int i = 0; i < n; ++i) {
            filtered[i] *= 0.55f + 0.45f * sinf(wam * (float)i);
        }
    }

    // Normalize to the requested peak amplitude, then convert to int16.
    float peak = 1e-6f;
    for (int i = 0; i < n; ++i) {
        float a = fabsf(filtered[i]);
        if (a > peak) peak = a;
    }
    float gain = amplitude / peak;
    out.resize(n);
    for (int i = 0; i < n; ++i) {
        float v = filtered[i] * gain;
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        out[i] = (int16_t)(v * 30000.0f);
    }
}

void AudioService::PlayCue(Cue cue) {
    int sample_rate = codec_ != nullptr ? codec_->output_sample_rate() : 24000;
    if (cue_sample_rate_ != sample_rate) {
        cue_turn_end_pcm_.clear();
        cue_processing_pcm_.clear();
        cue_sample_rate_ = sample_rate;
    }

    std::vector<int16_t>* pcm = nullptr;
    if (cue == Cue::kTurnEnd) {
        if (cue_turn_end_pcm_.empty()) {
            SynthSquelchCue(cue_turn_end_pcm_, sample_rate, 180, 0.5f, /*rasp=*/true);
        }
        pcm = &cue_turn_end_pcm_;
    } else {
        if (cue_processing_pcm_.empty()) {
            SynthSquelchCue(cue_processing_pcm_, sample_rate, 50, 0.3f, /*rasp=*/false);
        }
        pcm = &cue_processing_pcm_;
    }
    if (pcm != nullptr && !pcm->empty()) {
        PlayPcm(pcm->data(), pcm->size(), sample_rate);
    }
}

bool AudioService::IsIdle() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    return audio_encode_queue_.empty() && audio_decode_queue_.empty() && audio_playback_queue_.empty() && audio_testing_queue_.empty();
}

void AudioService::WaitForPlaybackQueueEmpty() {
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);
    audio_queue_cv_.wait(lock, [this]() { 
        return service_stopped_ || (audio_decode_queue_.empty() && audio_playback_queue_.empty()); 
    });
}

void AudioService::ResetDecoder() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
    if (opus_decoder_ != nullptr) {
        esp_opus_dec_reset(opus_decoder_);
    }
    decoder_lock.unlock();
    timestamp_queue_.clear();
    audio_decode_queue_.clear();
    audio_playback_queue_.clear();
    audio_testing_queue_.clear();
    audio_queue_cv_.notify_all();
}

void AudioService::CheckAndUpdateAudioPowerState() {
    auto now = std::chrono::steady_clock::now();
    auto input_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_input_time_).count();
    auto output_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_output_time_).count();
    if (input_elapsed > AUDIO_POWER_TIMEOUT_MS && codec_->input_enabled()) {
        codec_->EnableInput(false);
    }
    if (output_elapsed > AUDIO_POWER_TIMEOUT_MS && codec_->output_enabled()) {
        // Keep TX clock when duplex RX is active; otherwise RX may stall on some boards.
        if (!(codec_->duplex() && codec_->input_enabled())) {
            codec_->EnableOutput(false);
        }
    }
    if (!codec_->input_enabled() && !codec_->output_enabled()) {
        esp_timer_stop(audio_power_timer_);
    }
}

void AudioService::SetModelsList(srmodel_list_t* models_list) {
    models_list_ = models_list;

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
    if (esp_srmodel_filter(models_list_, ESP_MN_PREFIX, NULL) != nullptr) {
        wake_word_ = std::make_unique<CustomWakeWord>();
    } else if (esp_srmodel_filter(models_list_, ESP_WN_PREFIX, NULL) != nullptr) {
        wake_word_ = std::make_unique<AfeWakeWord>();
    } else {
        wake_word_ = nullptr;
    }
#else
    if (esp_srmodel_filter(models_list_, ESP_WN_PREFIX, NULL) != nullptr) {
        wake_word_ = std::make_unique<EspWakeWord>();
    } else {
        wake_word_ = nullptr;
    }
#endif

    if (wake_word_) {
        wake_word_->OnWakeWordDetected([this](const std::string& wake_word) {
            if (callbacks_.on_wake_word_detected) {
                callbacks_.on_wake_word_detected(wake_word);
            }
        });
    }
}

bool AudioService::IsAfeWakeWord() {
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
    return wake_word_ != nullptr && dynamic_cast<AfeWakeWord*>(wake_word_.get()) != nullptr;
#else
    return false;
#endif
}
