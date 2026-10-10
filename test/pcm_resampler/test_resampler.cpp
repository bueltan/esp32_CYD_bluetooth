#include "PcmResampler.h"
#include <cassert>
#include <cstdio>
#include <vector>

static std::vector<int16_t> convert(const std::vector<int16_t>& input, size_t block) {
  PcmResampler converter;
  std::vector<int16_t> result;
  for (size_t pos = 0; pos < input.size(); pos += block) {
    const size_t count = input.size() - pos < block ? input.size() - pos : block;
    const size_t capacity = PcmResampler::maxFrames(count) * 2;
    std::vector<int16_t> output(capacity + 2, 12345);
    const size_t frames = converter.process(input.data() + pos, count, output.data());
    assert(frames * 2 <= capacity);
    assert(output[capacity] == 12345 && output[capacity + 1] == 12345);
    result.insert(result.end(), output.begin(), output.begin() + frames * 2);
  }
  return result;
}
int main() {
  // 60 seconds + repeated endpoint: exactly 2,646,000 stereo frames.
  std::vector<int16_t> signal(960001);
  for (size_t i = 0; i < signal.size(); ++i)
    signal[i] = static_cast<int16_t>((i * 97) % 65536 - 32768);
  const auto whole = convert(signal, signal.size());
  const auto chunks = convert(signal, 320);
  assert(whole == chunks);
  assert(whole.size() == 2646000 * 2);
  assert(convert(signal, 1) == whole);
  assert(convert(signal, 127) == whole);
  for (size_t i = 0; i < whole.size(); i += 2) assert(whole[i] == whole[i + 1]);
  const auto constant = convert(std::vector<int16_t>(16001, -12345), 320);
  assert(constant.size() == 44100 * 2);
  for (int16_t sample : constant) assert(sample == -12345);
  puts("PASS: duration, block continuity, stereo, DC, output bounds");
}
