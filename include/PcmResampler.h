#pragma once
#include <stddef.h>
#include <stdint.h>

// Mono s16 at 16 kHz -> interleaved stereo s16 at 44.1 kHz.
// Preserve state across PCM blocks to avoid discontinuities and clock drift.
class PcmResampler {
 public:
  static constexpr uint32_t inputRate = 16000, outputRate = 44100;
  static constexpr size_t maxFrames(size_t samples) {
    return (samples * outputRate + inputRate - 1) / inputRate;
  }
  // Reserve maxFrames(count) * 2 int16_t values in output.
  size_t process(const int16_t* input, size_t count, int16_t* output) {
    size_t frames = 0;
    for (size_t i = 0; i < count; ++i) {
      const int16_t current = input[i];
      if (!primed_) {
        previous_ = current;
        primed_ = true;
        continue;
      }
      while (phase_ < outputRate) {
        const int32_t value = previous_ +
            static_cast<int64_t>(static_cast<int32_t>(current) - previous_) *
                phase_ / outputRate;
        output[frames * 2] = output[frames * 2 + 1] = static_cast<int16_t>(value);
        ++frames;
        phase_ += inputRate;
      }
      phase_ -= outputRate;
      previous_ = current;
    }
    return frames;
  }
 private:
  int16_t previous_ = 0;
  uint32_t phase_ = 0;
  bool primed_ = false;
};
