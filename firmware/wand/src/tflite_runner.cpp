#include "tflite_runner.h"

#include <TensorFlowLite_ESP32.h>

#include "magic_wand_model_data.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace {

constexpr int kRasterWidth = 32;
constexpr int kRasterHeight = 32;
constexpr int kRasterChannels = 3;
constexpr int kRasterByteCount = kRasterWidth * kRasterHeight * kRasterChannels;
constexpr int kLabelCount = 10;
constexpr int kTensorArenaSize = 30 * 1024;

const char* kLabels[kLabelCount] = {"0", "1", "2", "3", "4",
                                    "5", "6", "7", "8", "9"};

uint8_t tensor_arena[kTensorArenaSize];
tflite::MicroInterpreter* interpreter = nullptr;

}  // namespace

bool TfliteRunnerBegin() {
  static tflite::MicroErrorReporter micro_error_reporter;
  static tflite::MicroMutableOpResolver<4> micro_op_resolver;
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddMean();
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(
      tflite::GetModel(g_magic_wand_model_data), micro_op_resolver, tensor_arena,
      kTensorArenaSize, &micro_error_reporter);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    return false;
  }

  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != kRasterHeight) ||
      (model_input->dims->data[2] != kRasterWidth) ||
      (model_input->dims->data[3] != kRasterChannels) ||
      (model_input->type != kTfLiteInt8)) {
    return false;
  }

  TfLiteTensor* model_output = interpreter->output(0);
  if ((model_output->dims->size != 2) || (model_output->dims->data[0] != 1) ||
      (model_output->dims->data[1] != kLabelCount) ||
      (model_output->type != kTfLiteInt8)) {
    return false;
  }

  return true;
}

bool TfliteRunnerClassify(const int8_t* raster, int raster_bytes, const char** label,
                          int8_t* score) {
  if ((interpreter == nullptr) || (raster == nullptr) || (label == nullptr) ||
      (score == nullptr) || (raster_bytes != kRasterByteCount)) {
    return false;
  }

  TfLiteTensor* model_input = interpreter->input(0);
  for (int i = 0; i < kRasterByteCount; ++i) {
    model_input->data.int8[i] = raster[i];
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    return false;
  }

  TfLiteTensor* output = interpreter->output(0);
  int8_t max_score = output->data.int8[0];
  int max_index = 0;
  for (int i = 1; i < kLabelCount; ++i) {
    const int8_t candidate = output->data.int8[i];
    if (candidate > max_score) {
      max_score = candidate;
      max_index = i;
    }
  }

  *label = kLabels[max_index];
  *score = max_score;
  return true;
}
