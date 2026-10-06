/**
 * @file BTAudioBridge.cpp
 * @brief Implementation of Bluetooth A2DP audio receiver and real-time DSP pipeline for VESC motor coil sound.
 * @details Implements true-mono downmixing, Spotify-style asymmetric EMA peak envelope tracking, rational
 *          soft-knee dynamic range compression, cubic Hermite resampling, and zero-allocation 8-bit quantization.
 * @date 2026-08-25
 */

#include "BTAudioBridge.h"

BTAudioBridge* BTAudioBridge::active_instance = nullptr;

/**
 * @brief Constructs a new BTAudioBridge instance and creates the synchronization mutex.
 */
BTAudioBridge::BTAudioBridge() {
    active_instance = this;
    resampler_mutex = xSemaphoreCreateMutex();
}

/**
 * @brief Tears down the Bluetooth sink, deletes mutexes, and frees resources.
 */
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

/**
 * @brief Initializes Bluetooth A2DP Sink, sets audio pipeline target rate, and starts pairing listener.
 * @param[in] bt_name Human-readable Bluetooth broadcast SSID (e.g. "Kukirin Audio").
 * @param[in] target_sample_rate Target sample rate for motor FOC sound playback (e.g. 16000 Hz).
 * @param[in] output_cb Callback receiving processed 8-bit PCM samples to stream to VESC.
 * @note Core affinity: Core 0.
 */
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

/**
 * @brief Checks if a Bluetooth smartphone or audio source is actively connected.
 * @return bool True if A2DP connection is active; false otherwise.
 */
bool BTAudioBridge::is_connected() {
    return a2dp_sink.is_connected(); 
}

/**
 * @brief Checks if audio streaming is currently active and unpaused.
 * @return bool True if audio stream is running and producing PCM output; false otherwise.
 */
bool BTAudioBridge::is_playing() {
    return _is_playing && a2dp_sink.is_output_active();
}

#ifdef DEBUG
    /**
     * @brief Prints detailed Bluetooth and audio DSP diagnostics to Serial console.
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

// --- DSP PIPELINE ---

/**
 * @brief Updates internal resampler configurations when source sample rate changes.
 * @note Bounded mutex timeout (50ms) to prevent Core 0 watchdog timeout.
 */
void BTAudioBridge::update_resampler_config() {
    if (resampler_mutex == nullptr) return;

    // Fast acquisition attempt to prevent resource contention
    if (xSemaphoreTake(resampler_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        audio_tools::ResampleConfig rcfg = resampler.defaultConfig();
        rcfg.copyFrom(in_config);
        rcfg.to_sample_rate = out_config.sample_rate;
        resampler.begin(rcfg);
        xSemaphoreGive(resampler_mutex);
    }
}

/**
 * @brief Normalizes, mono-downmixes, soft-clips, and pumps audio frames into resampler.
 * @param[in] data Pointer to raw interleaved stereo 16-bit PCM bytes.
 * @param[in] length Length in bytes of the incoming data block.
 * @note Zero heap allocation. Uses static dsp_buffer.
 */
void BTAudioBridge::apply_normalization_and_scale(const uint8_t *data, uint32_t length) {
    if (!_is_playing || length == 0 || resampler_mutex == nullptr) return;
    
    const int16_t *src_pcm = reinterpret_cast<const int16_t*>(data);
    int16_t *dst_pcm = reinterpret_cast<int16_t*>(dsp_buffer);
    
    int num_frames = static_cast<int>(length / 4); 
    
    if ((num_frames * 2) > static_cast<int>(sizeof(dsp_buffer))) {
        num_frames = sizeof(dsp_buffer) / 2; 
    }

    int16_t current_buffer_peak = 1; 

    // Downmix to true-mono and detect instantaneous peak
    for (int i = 0; i < num_frames; i++) {
        int32_t mixed = (static_cast<int32_t>(src_pcm[i * 2]) + static_cast<int32_t>(src_pcm[i * 2 + 1])) / 2;
        dst_pcm[i] = static_cast<int16_t>(mixed); 
        
        int16_t val = abs(dst_pcm[i]);
        if (val > current_buffer_peak) current_buffer_peak = val;
    }

    // Envelope follower tracking via exponential moving average (EMA)
    const float attack_alpha = 0.20f;   // Fast attack (~50ms)
    const float release_alpha = 0.001f; // Ultra-slow release (~4-5 seconds)

    if (static_cast<float>(current_buffer_peak) > smoothed_peak) {
        // Audio got louder: track upward quickly to prevent harsh clipping
        smoothed_peak += attack_alpha * (static_cast<float>(current_buffer_peak) - smoothed_peak);
    } else {
        // Audio got quieter: decay slowly so quiet segments stay naturally quiet
        smoothed_peak += release_alpha * (static_cast<float>(current_buffer_peak) - smoothed_peak);
    }

    // Guard rails: prevent tracking envelope collapse into pure noise during silence
    if (smoothed_peak < 200.0f) smoothed_peak = 200.0f;

    // Calculate normalized gain multiplier
    float norm_multiplier = 32767.0f / smoothed_peak;
    
    // Cap the maximum possible boost to 8.0x to avoid amplifying background static/hiss
    if (norm_multiplier > 8.0f) norm_multiplier = 8.0f; 

    // Apply uniform gain and soft-knee compression clipping
    // Smooth audio compression begins once audio hits 80% of full scale
    const float threshold = 26214.0f; // 80% of 32767
    const float max_val = 32767.0f;
    const float knee_room = max_val - threshold; // 6553 dynamic units of "cushion"

    for (int i = 0; i < num_frames; i++) {
        float sample = static_cast<float>(dst_pcm[i]) * norm_multiplier;
        
        if (sample > threshold) {
            // Smoothly curve positive peaks toward 32767 using an asymptotic rational function
            float overshoot = sample - threshold;
            sample = threshold + (overshoot / (1.0f + (overshoot / knee_room)));
        } 
        else if (sample < -threshold) {
            // Smoothly curve negative peaks toward -32768
            float overshoot = -sample - threshold;
            sample = -threshold - (overshoot / (1.0f + (overshoot / knee_room)));
        }
        
        // Final safety cast (guaranteed to never exceed boundaries now)
        dst_pcm[i] = static_cast<int16_t>(sample);
    }

    // Avoid portMAX_DELAY to protect Core 0 Watchdog. 
    // Drop the frame if the resampler is temporarily busy being reconfigured.
    if (xSemaphoreTake(resampler_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        resampler.write(dsp_buffer, num_frames * 2);
        xSemaphoreGive(resampler_mutex);
    }
}

/**
 * @brief Converts resampled 16-bit PCM data to 8-bit signed format and dispatches via callback.
 * @param[in] buffer Pointer to 16-bit PCM byte array.
 * @param[in] size Total size in bytes of the buffer.
 * @return size_t Number of bytes processed.
 */
size_t BTAudioBridge::QuantizedAudioSink::write(const uint8_t *buffer, size_t size) {
    if (_parent == nullptr || buffer == nullptr || size == 0) return 0;

    const int16_t *pcm_16 = reinterpret_cast<const int16_t*>(buffer);
    size_t total_samples = size / 2;
    size_t samples_processed = 0;

    // Process all samples in bounded chunks without heap allocation or truncation
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

// --- STATIC CALLBACKS ---

/**
 * @brief Static callback invoked by BluetoothA2DPSink when SBC audio frames arrive.
 * @param[in] data Interleaved 16-bit stereo PCM stream pointer.
 * @param[in] length Byte count of audio chunk.
 * @note Core affinity: Core 0 (A2DP stack thread). Zero dynamic memory allocation.
 */
void BTAudioBridge::raw_audio_callback(const uint8_t *data, uint32_t length) {
    if (active_instance != nullptr)
        active_instance->apply_normalization_and_scale(data, length);
}

/**
 * @brief Static callback invoked on Bluetooth connection state transitions.
 * @param[in] state New connection state.
 * @param[in] obj Pointer to BTAudioBridge instance.
 * @note Core affinity: Core 0 (A2DP callback).
 */
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

/**
 * @brief Static callback invoked on Bluetooth audio streaming state transitions.
 * @param[in] state New audio state (STARTED, STOPPED, etc.).
 * @param[in] obj Pointer to BTAudioBridge instance.
 * @note Core affinity: Core 0 (A2DP callback).
 */
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

/**
 * @brief Static callback invoked when smartphone changes volume via AVRCP.
 * @param[in] volume Raw AVRCP volume (0 to 127).
 * @note Core affinity: Core 0 (AVRCP callback).
 */
void BTAudioBridge::volume_change_callback(int volume) {
    if (active_instance != nullptr) {
        active_instance->current_phone_volume = static_cast<uint8_t>(volume);
        active_instance->current_volume_percent = static_cast<uint8_t>((volume * 100) / 127);
    }

    #ifdef DEBUG
        Serial.printf("volume_change_callback: volume: %d\n", volume);
    #endif
}

/**
 * @brief Static callback invoked when smartphone negotiates a new sample rate (e.g. 44.1kHz vs 48kHz).
 * @param[in] new_rate Source sample rate in Hz. Valid range: 8000 to 96000.
 * @note Core affinity: Core 0 (A2DP callback).
 */
void BTAudioBridge::sample_rate_callback(uint16_t new_rate) {
    if (active_instance != nullptr && active_instance->in_config.sample_rate != new_rate && new_rate > 0) {
        active_instance->in_config.sample_rate = new_rate;
        active_instance->update_resampler_config();
    }

    #ifdef DEBUG
        Serial.printf("sample_rate_callback: new_rate: %d\n", new_rate);
    #endif
}