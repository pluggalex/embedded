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


typedef struct{
  s32 targets[3];
  u8 direction;// 0 = unknown, 1 = up, 2 = down 
}plannerHelper;

plannerHelper helper; //Global helper object to keep track of direction and current targets 

int existsInPlanner(val){
  for(int i = 0; i < 3; i++){
    if(helper.targets[i] == val)
      return 1;
  }
  return 0;
}

void requestPosition(s32 data){
  if(existsInPlanner(data)) return;

  s32 currentPosition = getCarPosition();
  s32 duty = getMotorCurrentDuty(); // Probably 200 duty per 1cm/s. Check motor.h 
  s32 stoppingDistance = 2 * duty / 200; // duty / 200 -> cm's per second. 2 == senconds given to stop
  
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
