#ifndef TARGET_LED_CONTROLLER_H_
#define TARGET_LED_CONTROLLER_H_

void LedControllerBegin();
void LedControllerLoop();

void LedControllerStartActivationFlash();
void LedControllerStop();
bool LedControllerIsFlashing();

#endif  // TARGET_LED_CONTROLLER_H_
