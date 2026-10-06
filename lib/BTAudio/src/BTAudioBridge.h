#pragma once

#include <Arduino.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

/**
 * @brief Receives a chunk of signed 8-bit mono samples (the format VESC foc-play-samples accepts).
 *        Called from the Bluetooth audio task; must not block.
 */
typedef void (*AudioOutputCallback)(const int8_t* pcm_data, size_t length);

/**
 * @class BTAudioBridge
 * @brief Planned feature, not used by the firmware: plays phone audio through the motor coils.
 *
 * Receives Bluetooth A2DP audio, mixes the two channels to mono, levels the volume (a peak follower with fast
 * attack and slow release, plus soft limiting near full scale), resamples to the target rate, and converts to
 * signed 8-bit samples for the output callback. Runs in the Bluetooth stack's task; buffers are fixed size.
 */
class BTAudioBridge {
public:
    /** @brief Creates the resampler mutex. */
    BTAudioBridge();

    /** @brief Stops the Bluetooth sink and deletes the mutex. */
    ~BTAudioBridge();

    /**
     * @brief Starts the Bluetooth audio sink.
     * @param[in] bt_name Name the phone sees when pairing.
     * @param[in] target_sample_rate Output sample rate in Hz.
     * @param[in] output_cb Receives the converted samples.
     */
    void init(const char* bt_name, uint16_t target_sample_rate, AudioOutputCallback output_cb);

    /** @brief True while a phone is connected. */
    bool is_connected();

    /** @brief True while audio is playing. */
    bool is_playing();

    /** @brief Phone volume, 0..100 %. */
    int get_volume_percent() const { return current_volume_percent; }
    
    #ifdef DEBUG
        /** @brief Prints connection and audio state to Serial. */
        void print_dbg();
    #endif

private:
    float smoothed_peak = 16384.0f;                                     /**< @brief Smoothed peak level used for the volume leveling. */
    
    static BTAudioBridge* active_instance;                              /**< @brief Instance used by the static library callbacks (one instance only). */

    BluetoothA2DPSink a2dp_sink;                                        /**< @brief ESP32-A2DP sink. */
    A2DPNoVolumeControl no_volume_control;                              /**< @brief Phone volume is not applied to the samples. */

    SemaphoreHandle_t resampler_mutex = nullptr;                        /**< @brief Guards the resampler while its rate changes. */
    AudioOutputCallback on_audio_out = nullptr;                         /**< @brief Output callback. */

    /** @brief Receives the resampled 16-bit samples and converts them to 8-bit for the callback. */
    class QuantizedAudioSink : public Print {
    public:

        explicit QuantizedAudioSink(BTAudioBridge* parent) : _parent(parent) {}

        /** @brief Required by Print; unused. */
        size_t write(uint8_t c) override { (void)c; return 1; }

        /** @brief Converts 16-bit samples to 8-bit in chunks and passes each chunk to the callback. */
        size_t write(const uint8_t *buffer, size_t size) override;

    private:
        BTAudioBridge* _parent = nullptr;                               /**< @brief Owner. */
    } final_sink{this};

    audio_tools::ResampleStreamT<audio_tools::HermiteInterpolator> resampler{final_sink}; /**< @brief Resampler (Hermite interpolation). */
    
    audio_tools::AudioInfo in_config{44100, 1, 16};                     /**< @brief Input format; the rate follows the phone. */
    audio_tools::AudioInfo out_config{16000, 1, 16};                    /**< @brief Output format; the rate is set in init(). */

    // Fixed buffers
    alignas(4) uint8_t dsp_buffer[4096];                                 /**< @brief Mono mix and leveling. */
    alignas(4) int8_t quantization_buffer[2048];                         /**< @brief 8-bit output chunk. */

    volatile uint8_t current_phone_volume = 0;                          /**< @brief Phone volume, 0..127. */
    volatile uint8_t current_volume_percent = 0;                         /**< @brief Phone volume, 0..100 %. */
    
    volatile bool _is_playing = false;                                  /**< @brief Audio playing. */
    volatile bool _is_connected = false;                                /**< @brief Phone connected. */

    /** @brief Reconfigures the resampler when the phone changes sample rate (waits at most 50 ms for the mutex). */
    void update_resampler_config();

    /** @brief Mixes to mono, levels and limits a block of stereo 16-bit samples, then feeds the resampler. */
    void apply_normalization_and_scale(const uint8_t *data, uint32_t length);

    // Callbacks registered with the ESP32-A2DP library
    static void raw_audio_callback(const uint8_t *data, uint32_t length);
    static void connection_state_callback(esp_a2d_connection_state_t state, void *obj);
    static void audio_state_callback(esp_a2d_audio_state_t state, void *obj);
    static void volume_change_callback(int volume);
    static void sample_rate_callback(uint16_t new_sample_rate);
};