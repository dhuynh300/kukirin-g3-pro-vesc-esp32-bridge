# BTAudio (planned, not in the firmware)

Bluetooth A2DP receiver intended to play phone audio through the motors: it receives the stereo stream, mixes it to mono, compresses and resamples it, and hands 8-bit PCM chunks to a callback for sending to the VESC.

Status: the library compiles, but the bridge firmware does not include it and it has not been tested on the scooter. It is kept for a future release.

Dependencies (declared in `library.json`, fetched only when the library is used): ESP32-A2DP and arduino-audio-tools by Phil Schatzmann, pinned to specific commits.
