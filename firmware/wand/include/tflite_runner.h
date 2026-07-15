#ifndef MAGIC_WAND_TFLITE_RUNNER_H_
#define MAGIC_WAND_TFLITE_RUNNER_H_

#include <cstdint>

// Initializes TFLite Micro and loads the embedded magic wand model.
bool TfliteRunnerBegin();

// Runs inference on a 32x32x3 int8 raster buffer (3072 bytes).
// Returns false on failure. Writes digit label ("0"-"9") and int8 confidence.
bool TfliteRunnerClassify(const int8_t* raster, int raster_bytes, const char** label,
                          int8_t* score);

#endif  // MAGIC_WAND_TFLITE_RUNNER_H_
