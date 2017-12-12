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
  u8 targetLength;
  u8 numberOfTargets;
  s32 targets[3];
  u8 direction;// 0 = unknown, 1 = up, 2 = down 
}plannerHelper;

plannerHelper helper = {3,0,{0},1}; //Global helper object to keep track of direction and current targets 

int cmpLessThan(const void *a, const void *b){
  return (*(s32*)a) < (*(s32*)b);
}

int cmpGreaterThan(const void *a, const void *b){
  return (*(s32*)a) > (*(s32*)b);
}

void sortTargets(int (*comparer)(const void *, const void*)){
    qsort(helper.targets, helper.numberOfTargets, sizeof(s32), comparer);
}

int existsInPlanner(s32 val){
  int i = 0;
  for(; i < helper.targetLength; i++){
    if(helper.targets[i] == val)
      return 1;
  }
  return 0;
}

void insertTarget(s32 stoppingDistance, s32 currentPosition, s32 data, int (*comparer)(const void *, const void*)){
  s32 minimumTarget = currentPosition;
  if(helper.direction == 1) minimumTarget = currentPosition + stoppingDistance;
  else minimumTarget = currentPosition - stoppingDistance;

  if (comparer(&(data), &(minimumTarget))){
    helper.targets[helper.numberOfTargets] = data;
    helper.numberOfTargets++;
    sortTargets(comparer);
  }
  else{
    sortTargets(comparer);
    helper.targets[helper.numberOfTargets] = data;
    helper.numberOfTargets++;
  }
}

void requestPosition(s32 data){
  s32 currentPosition = getCarPosition();
  s32 duty = getMotorCurrentDuty(); // Probably 200 duty per 1cm/s. Check motor.h 
  s32 stoppingDistance = 5;//TODO remove hardcoded 2 * duty / 200; // duty / 200 -> cm's per second. 2 == senconds given to stop
  int (*cmprFun[2]) (const void*, const void*);
  cmprFun[0] = cmpLessThan;
  cmprFun[1] = cmpGreaterThan;

  if(existsInPlanner(data)) return;
  insertTarget(stoppingDistance, currentPosition, data, cmprFun[helper.direction]);
}
        
u8 setNextTarget(){
  if (helper.numberOfTargets > 0){
    setCarTargetPosition(helper.targets[0]);
    return 1;
  }
  return 0;
}
        
void removeTarget(){
  if (helper.numberOfTargets > 0){
    helper.numberOfTargets--;
    memmove(&(helper.targets), &(helper.targets[1]), sizeof(s32)*helper.targetLength-1);
    helper.targets[helper.targetLength-1] = 0; 
  }
}

void decideDirection(s32 carPosition){
  if(carPosition > 39) helper.direction = 0;
  if(carPosition < 1) helper.direction = 1;
}

static void plannerTask(void *params) {
  PinEvent buffer;
  u8 floorSwitch = 1;

  while (1) {
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
      case(ARRIVED_AT_FLOOR) :
        decideDirection(getCarPosition());
        //Check that we not only arive at floor but also have stoped
        //Also, we will assume that this is the first target in targets
        //and remove the first taget. If this is not the case we are screwed.
        removeTarget(); 
        // TODO wait 1s before setting this flag to 1..
        floorSwitch = 1;
        break;
      case(LEFT_FLOOR) :
        break;
      default:
        break;
    }
    setNextTarget();
  }
}

void setupPlanner(unsigned portBASE_TYPE uxPriority) {
  xTaskCreate(plannerTask, "planner", 100, NULL, uxPriority, NULL);
}
