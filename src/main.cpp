#include <Arduino.h>
#include <BluetoothA2DPSource.h>
#include <esp_timer.h>
#include <atomic>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>
#include "PcmResampler.h"

extern const uint8_t rawStart[] asm("_binary_oct4_audio_half_raw_start");
extern const uint8_t rawEnd[] asm("_binary_oct4_audio_half_raw_end");
static constexpr char targetName[] = "WH-CH720N";
static constexpr size_t blockSamples = 320; // Simulated decoded Opus: 20 ms
static constexpr size_t queueBytes = 32 * 1024;
static constexpr size_t maxBlockBytes = PcmResampler::maxFrames(blockSamples) * 4;
BluetoothA2DPSource a2dp;
PcmResampler resampler;
StreamBufferHandle_t pcmQueue = nullptr;
std::atomic<bool> paused{false};
std::atomic<uint32_t> callbacks{0}, underruns{0}, silentFrames{0};
std::atomic<uint32_t> rawPosition{0}, fileLoops{0}, lateBlocks{0}, maxLateMs{0};
uint32_t nextBlock = 0;

// Wall durations include preemption: useful for deadlines, not CPU accounting.
struct Timing {
  uint32_t calls = 0, totalUs = 0, maxUs = 0, cores = 0;
};
Timing rawTiming, resampleTiming, callbackTiming;
portMUX_TYPE timingMux = portMUX_INITIALIZER_UNLOCKED;
void record(Timing& timing, int64_t start) {
  const uint32_t elapsed = static_cast<uint32_t>(esp_timer_get_time() - start);
  const uint32_t core = xPortGetCoreID();
  portENTER_CRITICAL(&timingMux);
  ++timing.calls;
  timing.totalUs += elapsed;
  if (elapsed > timing.maxUs) timing.maxUs = elapsed;
  timing.cores |= 1U << core;
  portEXIT_CRITICAL(&timingMux);
}
void printTiming(const char* name, Timing& timing) {
  portENTER_CRITICAL(&timingMux);
  const Timing snapshot = timing;
  timing = Timing{};
  portEXIT_CRITICAL(&timingMux);
  Serial.printf("TIME %s: n=%lu total=%.3f ms avg=%.3f ms max=%.3f ms cores=%s%s\n",
                name, static_cast<unsigned long>(snapshot.calls), snapshot.totalUs / 1000.0,
                snapshot.calls ? snapshot.totalUs / (1000.0 * snapshot.calls) : 0.0,
                snapshot.maxUs / 1000.0, snapshot.cores & 1 ? "0 " : "",
                snapshot.cores & 2 ? "1" : "");
}

// Integration boundary: only the producer submits mono s16 PCM at 16 kHz.
// Caller retries the same block on false. Resampler state is never reset at
// packet boundaries. Output is stereo s16 at 44100 Hz, as required by A2DP.
bool enqueueDecodedPcm(const int16_t* samples, size_t count) {
  static int16_t stereo[PcmResampler::maxFrames(blockSamples) * 2];
  if (count > blockSamples ||
      xStreamBufferSpacesAvailable(pcmQueue) < PcmResampler::maxFrames(count) * 4)
    return false;
  const int64_t start = esp_timer_get_time();
  const size_t frames = resampler.process(samples, count, stereo);
  record(resampleTiming, start);
  const size_t bytes = frames * 4;
  return xStreamBufferSend(pcmQueue, stereo, bytes, 0) == bytes;
}

bool produceRawBlock() {
  if (xStreamBufferSpacesAvailable(pcmQueue) < maxBlockBytes) return false;
  const int64_t start = esp_timer_get_time();
  int16_t mono[blockSamples];
  const size_t rawBytes = rawEnd - rawStart;
  size_t position = rawPosition;
  uint32_t loops = fileLoops;
  for (size_t i = 0; i < blockSamples; ++i) {
    const uint16_t bits = static_cast<uint16_t>(rawStart[position]) |
                         (static_cast<uint16_t>(rawStart[position + 1]) << 8);
    mono[i] = static_cast<int16_t>(bits);
    position += 2;
    if (position == rawBytes) { position = 0; ++loops; }
  }
  if (!enqueueDecodedPcm(mono, blockSamples)) return false;
  rawPosition = position;
  fileLoops = loops;
  record(rawTiming, start);
  return true;
}

// Single consumer; no decoding, allocation, logs, or blocking waits here.
int32_t getAudioData(uint8_t* data, int32_t len) {
  if (len <= 0) return 0;
  const int64_t start = esp_timer_get_time();
  callbacks.fetch_add(1, std::memory_order_relaxed);
  memset(data, 0, len);
  if (!paused.load(std::memory_order_relaxed)) {
    const size_t requested = static_cast<size_t>(len) & ~size_t(3);
    const size_t received = xStreamBufferReceive(pcmQueue, data, requested, 0);
    if (received < requested) {
      underruns.fetch_add(1, std::memory_order_relaxed);
      silentFrames.fetch_add((requested - received) / 4, std::memory_order_relaxed);
    }
  }
  record(callbackTiming, start);
  return len; // Silence pads shortages; never send a short PCM frame.
}

// CPU counters measure scheduled task runtime, unlike TIME wall durations.
// Static snapshots avoid allocating memory on every report.
void printCpuStats() {
#if configGENERATE_RUN_TIME_STATS && configUSE_TRACE_FACILITY
  static constexpr size_t capacity = 48;
  static TaskStatus_t tasks[capacity];
  struct Previous { UBaseType_t number; uint32_t runtime; };
  static Previous previous[capacity];
  static UBaseType_t previousCount = 0;
  static uint32_t previousTotal = 0;
  static bool primed = false;
  configRUN_TIME_COUNTER_TYPE total = 0;
  const UBaseType_t count = uxTaskGetSystemState(tasks, capacity, &total);
  if (!count) { Serial.println("CPU: snapshot insuficiente (mas de 48 tareas)"); return; }
  const uint32_t interval = static_cast<uint32_t>(total) - previousTotal;
  if (primed && interval) {
    Serial.printf("CPU ventana=%.1f ms (100%% = un core completo)\n", interval / 1000.0);
    for (UBaseType_t i = 0; i < count; ++i) {
      bool found = false;
      uint32_t delta = 0;
      for (UBaseType_t j = 0; j < previousCount; ++j) {
        if (previous[j].number == tasks[i].xTaskNumber) {
          delta = static_cast<uint32_t>(tasks[i].ulRunTimeCounter) - previous[j].runtime;
          found = true;
          break;
        }
      }
      if (!found) continue; // A new task needs a baseline first.
      const double percent = 100.0 * delta / interval;
      if (strncmp(tasks[i].pcTaskName, "IDLE", 4) == 0) {
        Serial.printf("CPU core=%ld ocupado~=%.1f%% idle=%.1f%%\n",
                      static_cast<long>(tasks[i].xCoreID),
                      100.0 - min(100.0, percent), percent);
      } else if (delta >= 500 || strcmp(tasks[i].pcTaskName, "loopTask") == 0) {
        Serial.printf("TASK %-16s afinidad=%ld cpu=%.3f ms (%.2f%%) stack_min=%u B\n",
                      tasks[i].pcTaskName,
                      tasks[i].xCoreID == tskNO_AFFINITY ? -1L : static_cast<long>(tasks[i].xCoreID),
                      delta / 1000.0, percent, static_cast<unsigned>(tasks[i].usStackHighWaterMark));
      }
    }
  }
  for (UBaseType_t i = 0; i < count; ++i)
    previous[i] = {tasks[i].xTaskNumber, static_cast<uint32_t>(tasks[i].ulRunTimeCounter)};
  previousCount = count;
  previousTotal = static_cast<uint32_t>(total);
  primed = true;
#else
  Serial.println("CPU: SDK sin estadisticas FreeRTOS; TIME solo mide duracion de pared");
#endif
}

void fatal(const char* message) {
  Serial.println(message);
  while (true) delay(1000);
}
void diagnosticsTask(void*);
void setup() {
  Serial.begin(115200);
  delay(1000);
  const size_t rawBytes = rawEnd - rawStart;
  Serial.println("\nRAW mono 16 kHz/s16le -> bloques 20 ms -> 44.1 kHz stereo -> A2DP");
  Serial.printf("RAW: %u bytes, %.2f s. loop core=%d\n", static_cast<unsigned>(rawBytes),
                rawBytes / 32000.0, xPortGetCoreID());
  Serial.printf("SDK: Bluedroid core=%d, controlador BT core=%d\n",
                CONFIG_BT_BLUEDROID_PINNED_TO_CORE, CONFIG_BTDM_CTRL_PINNED_TO_CORE);
  Serial.println("TIME incluye interrupciones/preemption; CPU usa runtime FreeRTOS.");
  Serial.println("RAW+cola incluye resample: no sumar ambos tiempos. Afinidad -1 = libre.");
  if (rawBytes < 4 || rawBytes % 2) fatal("ERROR: RAW vacio o s16 incompleto");
  pcmQueue = xStreamBufferCreate(queueBytes, 1);
  if (!pcmQueue) fatal("ERROR: no hay RAM para cola PCM");
  for (size_t i = 0; i < 6; ++i)
    if (!produceRawBlock()) fatal("ERROR: precarga PCM");
  a2dp.set_data_callback(getAudioData);
  a2dp.set_volume(80);
  a2dp.set_auto_reconnect(true, 5);
  Serial.printf("Conectando a %s. Comando: p = pausa/reanudar\n", targetName);
  a2dp.start(targetName);
  nextBlock = millis();
  if (xTaskCreatePinnedToCore(diagnosticsTask, "audio_diag", 4096, nullptr, 1, nullptr,
                              CONFIG_ARDUINO_RUNNING_CORE) != pdPASS)
    fatal("ERROR: tarea de diagnostico");
}
void loop() {
  if (Serial.available() && Serial.read() == 'p') {
    paused.store(!paused.load());
    Serial.println(paused.load() ? "Pausa" : "Reproduciendo");
  }
  const uint32_t now = millis();
  if (paused.load() || !a2dp.is_connected()) {
    nextBlock = now;
  } else if (static_cast<int32_t>(now - nextBlock) >= 0) {
    const uint32_t late = now - nextBlock;
    if (late > 2) ++lateBlocks;
    uint32_t previousLate = maxLateMs.load();
    while (late > previousLate && !maxLateMs.compare_exchange_weak(previousLate, late)) {}
    if (produceRawBlock()) {
      nextBlock += 20;
      if (static_cast<int32_t>(now - nextBlock) > 100) nextBlock = now + 20;
    } else nextBlock = now + 20;
  }
  delay(1);
}

void diagnosticsTask(void*) {
  printCpuStats();
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    const size_t queued = xStreamBufferBytesAvailable(pcmQueue);
    Serial.printf("\nAUDIO BT=%s pausa=%d RAW=%.2fs vueltas=%lu cola=%u (%.1fms) "
                  "callbacks=%lu underruns=%lu silencio=%lu frames heap=%u\n",
                  a2dp.is_connected() ? "conectado" : "esperando", paused.load(),
                  rawPosition.load() / 32000.0, static_cast<unsigned long>(fileLoops.load()),
                  static_cast<unsigned>(queued), queued * 1000.0 / (44100 * 4),
                  static_cast<unsigned long>(callbacks.load()),
                  static_cast<unsigned long>(underruns.load()),
                  static_cast<unsigned long>(silentFrames.load()), ESP.getFreeHeap());
    Serial.printf("SCHED bloques tarde (>2ms)=%lu atraso_max=%lu ms\n",
                  static_cast<unsigned long>(lateBlocks.exchange(0)),
                  static_cast<unsigned long>(maxLateMs.exchange(0)));
    printTiming("RAW+cola", rawTiming);
    printTiming("resample", resampleTiming);
    printTiming("callback", callbackTiming);
    printCpuStats();
  }
}
