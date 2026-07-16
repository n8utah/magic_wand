#ifndef WAND_LED_CONTROLLER_H_
#define WAND_LED_CONTROLLER_H_

#include <cstdint>

void LedControllerBegin();
void LedControllerLoop(bool is_recording_motion);

void LedControllerShowRecognitionResult(bool inference_ok, int8_t score,
                                        bool spell_activated);

bool LedControllerIsBusy();

#endif  // WAND_LED_CONTROLLER_H_
