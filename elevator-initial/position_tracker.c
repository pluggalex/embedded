/**
 * Program skeleton for the course "Programming embedded systems"
 *
 * Lab 1: the elevator control system
 */

/**
 * Class for keeping track of the car position.
 */

#include "FreeRTOS.h"
#include "task.h"

#include "position_tracker.h"

#include "assert.h"

//Params here will be the tracker
static void positionTrackerTask(void *params) {

  u8 pin_state;
  portTickType pollingPeriod;
  portTickType xLastWakeTime = xTaskGetTickCount();
  PositionTracker *tracker = (PositionTracker *)(params);
  tracker->status = Bit_RESET; //Inital "last status" of pin is 0
 
  for(;;) {
    xSemaphoreTake(tracker->lock, portMAX_DELAY);
    pin_state = GPIO_ReadInputDataBit(tracker->gpio, tracker->pin);

    if (pin_state == Bit_SET && tracker->status == Bit_RESET) {
      tracker->status = Bit_SET;
      if (tracker->direction == 1) tracker->position++;
      if (tracker->direction == 2) tracker->position--;
    } else if (pin_state == Bit_RESET && tracker->status != Bit_RESET) {
      tracker->status = Bit_RESET;
    }

    pollingPeriod = tracker->pollingPeriod;
    xSemaphoreGive(tracker->lock);
    vTaskDelayUntil(&xLastWakeTime, pollingPeriod);
  }
}

void setupPositionTracker(PositionTracker *tracker,
                          GPIO_TypeDef * gpio, u16 pin,
						  portTickType pollingPeriod,
						  unsigned portBASE_TYPE uxPriority) {
  portBASE_TYPE res;

  tracker->position = 0;
  tracker->lock = xSemaphoreCreateMutex();
  assert(tracker->lock != NULL);
  tracker->direction = Unknown;
  tracker->gpio = gpio;
  tracker->pin = pin;
  tracker->pollingPeriod = pollingPeriod;
  
  res = xTaskCreate(positionTrackerTask, "position tracker",
                    80, (void*)tracker, uxPriority, NULL);
  assert(res == pdTRUE);
}

void setDirection(PositionTracker *tracker, Direction dir) {
  // ..
  
  xSemaphoreTake(tracker->lock, portMAX_DELAY);
  tracker->direction = dir;
  xSemaphoreGive(tracker->lock);
}

s32 getPosition(PositionTracker *tracker) {
  s32 current_position; 

  xSemaphoreTake(tracker->lock, portMAX_DELAY);
  current_position = tracker->position;
  xSemaphoreGive(tracker->lock);
  // ...
  printf("current_position: %ld\n", current_position);
  return current_position;

}

Direction getDirection(PositionTracker *tracker) {
  s32 currentDirection; 

  xSemaphoreTake(tracker->lock, portMAX_DELAY);
  currentDirection = tracker->direction;
  xSemaphoreGive(tracker->lock);

  return currentDirection;
}

