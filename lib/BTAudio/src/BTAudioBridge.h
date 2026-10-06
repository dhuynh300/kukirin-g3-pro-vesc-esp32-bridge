#pragma once

#include <Arduino.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

/**
 * @brief Function pointer callback type for dispatching 8-bit quantized PCM audio frames to VESC.
 * @param[in] pcm_data Pointer to contiguous array of 8-bit signed PCM samples.
 * @param[in] length Number of 8-bit samples in the chunk.
 * @note Zero heap allocation. Callback must not block or call dynamic allocators.
 */
typedef void (*AudioOutputCallback)(const int8_t* pcm_data, size_t length);

/**
 * @class BTAudioBridge
 * @brief Bluetooth Classic A2DP audio receiver and real-time DSP pipeline for VESC motor coil sound synthesis.
 * @details Receives SBC A2DP stereo streams from paired smartphones, downmixes to mono, applies an asymmetric
 *          EMA peak envelope tracker with rational soft-knee compression, resamples to target sample rate via
 *          cubic Hermite interpolation, quantizes to 8-bit signed PCM, and dispatches to VESC FOC motor coils.
 * @note Core affinity: Core 0 (Bluetooth stack & DSP), dispatching to Core 1 (VESC UART).
 * @note Zero dynamic memory allocation in audio processing paths. Pre-allocated static buffers.
 */
class BTAudioBridge {
public:
    /**
     * @brief Constructs a new BTAudioBridge instance and creates the synchronization mutex.
     */
    BTAudioBridge();

    /**
     * @brief Tears down the Bluetooth sink, deletes mutexes, and frees resources.
     */
    ~BTAudioBridge();

    /**
     * @brief Initializes Bluetooth A2DP Sink, sets audio pipeline target rate, and starts pairing listener.
     * @param[in] bt_name Human-readable Bluetooth broadcast SSID (e.g. "Kukirin Audio").
     * @param[in] target_sample_rate Target sample rate for motor FOC sound playback (e.g. 16000 Hz).
     * @param[in] output_cb Callback receiving processed 8-bit PCM samples to stream to VESC.
     * @note Core affinity: Core 0.
     */
    void init(const char* bt_name, uint16_t target_sample_rate, AudioOutputCallback output_cb);

    /**
     * @brief Checks if a Bluetooth smartphone or audio source is actively connected.
     * @return bool True if A2DP connection is active; false otherwise.
     * @note Thread-safe. Core affinity: Core 0 / Core 1.
     */
    bool is_connected();

    /**
     * @brief Checks if audio streaming is currently active and unpaused.
     * @return bool True if audio stream is running and producing PCM output; false otherwise.
     * @note Thread-safe. Core affinity: Core 0 / Core 1.
     */
    bool is_playing();

    /**
     * @brief Gets current volume percentage received from smartphone AVRCP volume sync.
     * @return int Volume percentage (0 to 100%).
     * @note Thread-safe atomic read. Core affinity: Core 0 / Core 1.
     */
    int get_volume_percent() const { return current_volume_percent; }
    
    #ifdef DEBUG
        /**
         * @brief Prints detailed Bluetooth and audio DSP diagnostics to Serial console.
         * @note Core affinity: Core 0 / Core 1.
         */
        void print_dbg();
    #endif

private:
    float smoothed_peak = 16384.0f;                                     /**< @brief Asymmetric EMA smoothed audio peak level. */
    
    static BTAudioBridge* active_instance;                              /**< @brief Singleton instance pointer for static C callbacks. */

    BluetoothA2DPSink a2dp_sink;                                        /**< @brief ESP32-A2DP library sink handle. */
    A2DPNoVolumeControl no_volume_control;                              /**< @brief Disables software attenuation in A2DP stack. */

    SemaphoreHandle_t resampler_mutex = nullptr;                        /**< @brief Mutex protecting resampler configuration. */
    AudioOutputCallback on_audio_out = nullptr;                         /**< @brief Registered audio frame delivery callback. */

    /**
     * @class QuantizedAudioSink
     * @brief Custom Print sink receiving resampled 16-bit PCM and quantizing to 8-bit signed PCM.
     */
    class QuantizedAudioSink : public Print {
    public:
        /**
         * @brief Constructs a QuantizedAudioSink bound to its parent BTAudioBridge.
         * @param[in] parent Pointer to parent BTAudioBridge instance.
         */
        explicit QuantizedAudioSink(BTAudioBridge* parent) : _parent(parent) {}

        /**
         * @brief Character write stub satisfying the Print interface.
         * @param[in] c Character byte.
         * @return size_t Always returns 1.
         */
        size_t write(uint8_t c) override { (void)c; return 1; }

        /**
         * @brief Converts resampled 16-bit PCM data to 8-bit signed format and dispatches via callback.
         * @param[in] buffer Pointer to 16-bit PCM byte array.
         * @param[in] size Total size in bytes of the buffer.
         * @return size_t Number of bytes processed.
         * @note Zero heap allocation. Writes directly to pre-allocated quantization_buffer.
         */
        size_t write(const uint8_t *buffer, size_t size) override;

    private:
        BTAudioBridge* _parent = nullptr;                               /**< @brief Pointer to outer BTAudioBridge class. */
    } final_sink{this};

    audio_tools::ResampleStreamT<audio_tools::HermiteInterpolator> resampler{final_sink}; /**< @brief Cubic Hermite polynomial resampler. */
    
    audio_tools::AudioInfo in_config{44100, 1, 16};                     /**< @brief Resampler input stream configuration (44.1kHz mono 16-bit). */
    audio_tools::AudioInfo out_config{16000, 1, 16};                    /**< @brief Resampler output stream configuration (16kHz mono 16-bit). */

    // Pre-allocated static buffers (zero heap allocation in real-time path)
    alignas(4) uint8_t dsp_buffer[4096];                                 /**< @brief Static DSP scratchpad buffer for mono mixing and AGC (4-byte aligned). */
    alignas(4) int8_t quantization_buffer[2048];                         /**< @brief Static output buffer for quantized 8-bit PCM samples (4-byte aligned). */

    volatile uint8_t current_phone_volume = 0;                          /**< @brief Raw AVRCP volume (0 to 127). */
    volatile uint8_t current_volume_percent = 0;                         /**< @brief Normalized volume (0 to 100%). */
    
    volatile bool _is_playing = false;                                  /**< @brief Audio playback active flag. */
    volatile bool _is_connected = false;                                /**< @brief Bluetooth device connected flag. */

    /**
     * @brief Updates internal resampler configurations when source sample rate changes.
     * @note Bounded mutex timeout (50ms) to prevent Core 0 watchdog timeout.
     */
    void update_resampler_config();

    /**
     * @brief Normalizes, mono-downmixes, soft-clips, and pumps audio frames into resampler.
     * @param[in] data Pointer to raw interleaved stereo 16-bit PCM bytes.
     * @param[in] length Length in bytes of the incoming data block.
     * @note Zero heap allocation. Core affinity: Core 0 audio task.
     */
    void apply_normalization_and_scale(const uint8_t *data, uint32_t length);

    // Static C Callbacks registered with ESP32-A2DP library
    static void raw_audio_callback(const uint8_t *data, uint32_t length);
    static void connection_state_callback(esp_a2d_connection_state_t state, void *obj);
    static void audio_state_callback(esp_a2d_audio_state_t state, void *obj);
    static void volume_change_callback(int volume);
    static void sample_rate_callback(uint16_t new_sample_rate);
};