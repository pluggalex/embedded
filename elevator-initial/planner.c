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
#include <stdlib.h>
#include <string.h>

#include "global.h"
#include "planner.h"
#include "assert.h"


typedef struct{
  u8 queueLength;
  u8 numberOfTargets;
  s32 targets[3];
  u8 direction;// 0 = unknown, 1 = up, 2 = down 
}plannerHelper;

plannerHelper helper = {
  .queueLength = 3,
  .numberOfTargets = 0,
  .targets = {0},
  .direction = 1
}; //Global helper object to keep track of direction and current targets 

int cmpLessThan(const void *a, const void *b){
  return (*(s32*)a) < (*(s32*)b);
}

int cmpGreaterThan(const void *a, const void *b){
  return (*(s32*)a) > (*(s32*)b);
}

void sortTargets(){
  if(helper.direction == 1){ 
    qsort(helper.targets, helper.numberOfTargets, sizeof(s32), cmpLessThan);
  }
  else{
    qsort(helper.targets, helper.numberOfTargets, sizeof(s32), cmpGreaterThan);
  }
}

int existsInPlanner(s32 val){
  for(int i = 0; i < 3; i++){
    if(helper.targets[i] == val)
      return 1;
  }
  return 0;
}

void insertTarget(s32 stoppingDistance, s32 currentPosition, s32 data, int (*tooClose)(const void *, const void*)){
  s32 minimumTarget = currentPosition;
  if(helper.direction == 1) minimumTarget = currentPosition + stoppingDistance;
  else minimumTarget = currentPosition - stoppingDistance;

  if (tooClose(&(data), &(minimumTarget))){
    sortTargets();
    helper.targets[helper.numberOfTargets] = data;
    helper.numberOfTargets++;
  }
  else{
    helper.targets[helper.numberOfTargets] = data;
    helper.numberOfTargets++;
    sortTargets();
  }
}

void requestPosition(s32 data){
  if(existsInPlanner(data)) return;

  s32 currentPosition = getCarPosition();
  s32 duty = getMotorCurrentDuty(); // Probably 200 duty per 1cm/s. Check motor.h 
  s32 stoppingDistance = 2 * duty / 200; // duty / 200 -> cm's per second. 2 == senconds given to stop
  int (*cmprFun[2]) (const void*, const void*);
  cmprFun[0] = cmpGreaterThan;
  cmprFun[1] = cmpLessThan;

  insertTarget(stoppingDistance, currentPosition, data, cmprFun[helper.direction]);
}

void setDirection(s32 carPosition){
  if(carPosition > 799) helper.direction = 0;
  if(carPosition < 1) helper.direction = 1;
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
      case(ARRIVED_AT_FLOOR):
        //TODO Wait 1s
        setDirection(getCarPosition());
        break;
      default:
        break;
    }
  }
}

void setupPlanner(unsigned portBASE_TYPE uxPriority) {
  xTaskCreate(plannerTask, "planner", 100, NULL, uxPriority, NULL);
}
