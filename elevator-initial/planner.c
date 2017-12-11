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

static void plannerTask(void *params) {

  // ...
  while (1) {
    PinEvent buffer;
    xQueueReceive(pinEventQueue, &buffer, portMAX_DELAY);

    if (buffer == TO_FLOOR_1) {
      setCarTargetPosition(0);
    }
    if (buffer == TO_FLOOR_2) {
      setCarTargetPosition(400);
    }
    if (buffer == TO_FLOOR_3) {
      setCarTargetPosition(800);
    }
  }
}

void setupPlanner(unsigned portBASE_TYPE uxPriority) {
  xTaskCreate(plannerTask, "planner", 100, NULL, uxPriority, NULL);
}
