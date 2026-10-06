/**
 * @file BTAudioBridge.cpp
 * @brief Bluetooth audio to 8-bit samples for motor-coil playback (planned feature). See BTAudioBridge.h.
 */

#include "BTAudioBridge.h"

BTAudioBridge* BTAudioBridge::active_instance = nullptr;

BTAudioBridge::BTAudioBridge() {
    active_instance = this;
    resampler_mutex = xSemaphoreCreateMutex();
}

BTAudioBridge::~BTAudioBridge() {
    #ifdef DEBUG
        Serial.println("BtAudio:Teardown:Start");
    #endif
    
    a2dp_sink.end(); 
    active_instance = nullptr; 
    
    _is_playing = false;
    _is_connected = false;

    if (resampler_mutex != nullptr) {
        vSemaphoreDelete(resampler_mutex);
        resampler_mutex = nullptr;
    }
    
    #ifdef DEBUG
        Serial.println("BtAudio:Teardown:Complete");
    #endif
}

void BTAudioBridge::init(const char* bt_name, uint16_t target_sample_rate, AudioOutputCallback output_cb) {
    #ifdef DEBUG
        Serial.println("BtAudio:Init:Start");
    #endif

    out_config.sample_rate = target_sample_rate;
    on_audio_out = output_cb;

    update_resampler_config();

    a2dp_sink.set_on_connection_state_changed(connection_state_callback, this);
    a2dp_sink.set_on_audio_state_changed(audio_state_callback, this);
    a2dp_sink.set_on_volumechange(volume_change_callback);
    a2dp_sink.set_sample_rate_callback(sample_rate_callback);
    
    a2dp_sink.set_volume_control(&no_volume_control);
    a2dp_sink.set_stream_reader(raw_audio_callback, false);

    a2dp_sink.start(bt_name);
}

bool BTAudioBridge::is_connected() {
    return a2dp_sink.is_connected(); 
}

bool BTAudioBridge::is_playing() {
    return _is_playing && a2dp_sink.is_output_active();
}

#ifdef DEBUG
    /**
     * @brief Prints connection and audio state to Serial.
     */
    void BTAudioBridge::print_dbg() {
        Serial.println("====== BT AUDIO STATUS ======");
        Serial.printf("Connected: %s\n", a2dp_sink.is_connected() ? "YES" : "NO");
        if (a2dp_sink.is_connected()) {
            Serial.printf("Peer Name: %s\n", a2dp_sink.get_peer_name());
        }
        Serial.printf("Playing: %s\n", is_playing() ? "YES" : "NO");
        Serial.printf("Source Rate: %d Hz\n", a2dp_sink.sample_rate());
        Serial.printf("Target Rate: %d Hz\n", out_config.sample_rate);
        Serial.printf("Phone Volume: %d%% (%d/127)\n", get_volume_percent(), current_phone_volume);
        Serial.println("=============================");
    }
#endif

// Audio processing

void BTAudioBridge::update_resampler_config() {
    if (resampler_mutex == nullptr) return;

    if (xSemaphoreTake(resampler_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        audio_tools::ResampleConfig rcfg = resampler.defaultConfig();
        rcfg.copyFrom(in_config);
        rcfg.to_sample_rate = out_config.sample_rate;
        resampler.begin(rcfg);
        xSemaphoreGive(resampler_mutex);
    }
}

void BTAudioBridge::apply_normalization_and_scale(const uint8_t *data, uint32_t length) {
    if (!_is_playing || length == 0 || resampler_mutex == nullptr) return;
    
    const int16_t *src_pcm = reinterpret_cast<const int16_t*>(data);
    int16_t *dst_pcm = reinterpret_cast<int16_t*>(dsp_buffer);
    
    int num_frames = static_cast<int>(length / 4); 
    
    if ((num_frames * 2) > static_cast<int>(sizeof(dsp_buffer))) {
        num_frames = sizeof(dsp_buffer) / 2; 
    }

    int16_t current_buffer_peak = 1; 

    // Average the two channels and find the block's peak
    for (int i = 0; i < num_frames; i++) {
        int32_t mixed = (static_cast<int32_t>(src_pcm[i * 2]) + static_cast<int32_t>(src_pcm[i * 2 + 1])) / 2;
        dst_pcm[i] = static_cast<int16_t>(mixed); 
        
        int16_t val = abs(dst_pcm[i]);
        if (val > current_buffer_peak) current_buffer_peak = val;
    }

    // Peak follower: rises fast, falls slowly
    const float attack_alpha = 0.20f;
    const float release_alpha = 0.001f;

    if (static_cast<float>(current_buffer_peak) > smoothed_peak) {
        // Louder: follow quickly so peaks are not clipped
        smoothed_peak += attack_alpha * (static_cast<float>(current_buffer_peak) - smoothed_peak);
    } else {
        // Quieter: follow slowly so quiet passages stay quiet
        smoothed_peak += release_alpha * (static_cast<float>(current_buffer_peak) - smoothed_peak);
    }

    // Floor, so silence is not amplified into noise
    if (smoothed_peak < 200.0f) smoothed_peak = 200.0f;

    float norm_multiplier = 32767.0f / smoothed_peak;
    
    // At most 8x gain
    if (norm_multiplier > 8.0f) norm_multiplier = 8.0f; 

    // Gain, then soft limiting above 80 % of full scale
    const float threshold = 26214.0f; // 80% of 32767
    const float max_val = 32767.0f;
    const float knee_room = max_val - threshold;

    for (int i = 0; i < num_frames; i++) {
        float sample = static_cast<float>(dst_pcm[i]) * norm_multiplier;
        
        if (sample > threshold) {
            // Approach 32767 smoothly instead of clipping
            float overshoot = sample - threshold;
            sample = threshold + (overshoot / (1.0f + (overshoot / knee_room)));
        } 
        else if (sample < -threshold) {
            float overshoot = -sample - threshold;
            sample = -threshold - (overshoot / (1.0f + (overshoot / knee_room)));
        }
        
        dst_pcm[i] = static_cast<int16_t>(sample);
    }

    // If the resampler is being reconfigured, drop this block rather than wait
    if (xSemaphoreTake(resampler_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        resampler.write(dsp_buffer, num_frames * 2);
        xSemaphoreGive(resampler_mutex);
    }
}

/**
 * @brief 16-bit to 8-bit in chunks of quantization_buffer size.
 */
size_t BTAudioBridge::QuantizedAudioSink::write(const uint8_t *buffer, size_t size) {
    if (_parent == nullptr || buffer == nullptr || size == 0) return 0;

    const int16_t *pcm_16 = reinterpret_cast<const int16_t*>(buffer);
    size_t total_samples = size / 2;
    size_t samples_processed = 0;

    while (samples_processed < total_samples) {
        size_t chunk = total_samples - samples_processed;
        if (chunk > sizeof(_parent->quantization_buffer)) {
            chunk = sizeof(_parent->quantization_buffer);
        }

        for (size_t i = 0; i < chunk; i++) {
            _parent->quantization_buffer[i] = static_cast<int8_t>(pcm_16[samples_processed + i] >> 8);
        }

        if (_parent->on_audio_out != nullptr) {
            _parent->on_audio_out(_parent->quantization_buffer, chunk);
        }

        samples_processed += chunk;
    }
    
    return size; 
}

// Library callbacks

/** @brief Decoded audio from the phone: interleaved stereo 16-bit. */
void BTAudioBridge::raw_audio_callback(const uint8_t *data, uint32_t length) {
    if (active_instance != nullptr)
        active_instance->apply_normalization_and_scale(data, length);
}

/** @brief Connection state changed. */
void BTAudioBridge::connection_state_callback(esp_a2d_connection_state_t state, void *obj) {
    BTAudioBridge* bridge = static_cast<BTAudioBridge*>(obj);
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        bridge->_is_connected = true;
    }
    else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        bridge->_is_connected = false;
    }

    #ifdef DEBUG
        Serial.printf("connection_state_callback: state: %d=%s\n", state, bridge->a2dp_sink.to_str(state));
    #endif
}

/** @brief Playback started or stopped. */
void BTAudioBridge::audio_state_callback(esp_a2d_audio_state_t state, void *obj) {
    BTAudioBridge* bridge = static_cast<BTAudioBridge*>(obj);
    if (state == ESP_A2D_AUDIO_STATE_STARTED) {
        bridge->_is_playing = true;
        
        int real_volume = bridge->a2dp_sink.get_volume();
        bridge->current_phone_volume = static_cast<uint8_t>(real_volume);
        bridge->current_volume_percent = static_cast<uint8_t>((real_volume * 100) / 127);
    }
    else {
        bridge->_is_playing = false;
    }

    #ifdef DEBUG
        Serial.printf("audio_state_callback: state: %d=%s\n", state, bridge->a2dp_sink.to_str(state));
    #endif
}

/** @brief Phone volume changed (0..127). */
void BTAudioBridge::volume_change_callback(int volume) {
    if (active_instance != nullptr) {
        active_instance->current_phone_volume = static_cast<uint8_t>(volume);
        active_instance->current_volume_percent = static_cast<uint8_t>((volume * 100) / 127);
    }

    #ifdef DEBUG
        Serial.printf("volume_change_callback: volume: %d\n", volume);
    #endif
}

/** @brief Phone sample rate changed (for example 44.1 kHz to 48 kHz). */
void BTAudioBridge::sample_rate_callback(uint16_t new_rate) {
    if (active_instance != nullptr && active_instance->in_config.sample_rate != new_rate && new_rate > 0) {
        active_instance->in_config.sample_rate = new_rate;
        active_instance->update_resampler_config();
    }

    #ifdef DEBUG
        Serial.printf("sample_rate_callback: new_rate: %d\n", new_rate);
    #endif
}