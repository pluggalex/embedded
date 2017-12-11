/**
 * Program skeleton for the course "Programming embedded systems"
 *
 * Lab 1: the elevator control system
 */

/**
 * The planner module, which is responsible for consuming
 * pin/key events, and for deciding where the elevator
 * should go next
 */

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "global.h"
#include "planner.h"
#include "assert.h"

struct{
  s32 targets[3];
  u8 direction;
}plannerHelper;

void requestPosition(s32 data){
  s32 currentPosition = getCarPosition();
  setCarTargetPosition(data);
}

static void plannerTask(void *params) {

  // ...
  while (1) {
    PinEvent buffer;
    xQueueReceive(pinEventQueue, &buffer, portMAX_DELAY);

    switch (buffer) {
      case(TO_FLOOR_1) :
        requestPosition(0);
        break;
      case(TO_FLOOR_2) :
        requestPosition(20);
        break;
      case(TO_FLOOR_3) :
        requestPosition(40);
        break;
      default:
        break;
    }
  }
}

void setupPlanner(unsigned portBASE_TYPE uxPriority) {
  xTaskCreate(plannerTask, "planner", 100, NULL, uxPriority, NULL);
}
