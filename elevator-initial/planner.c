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
#include "semphr.h"

#include "global.h"
#include "planner.h"
#include "assert.h"

typedef enum{
  ARRIVE = 0,
  LEAVE = 1
}floorEvent;

typedef enum{
  OPEN = 0,
  CLOSED = 1
}doorEvent;

typedef enum{
  DOWN = 0,
  UP = 1
}directionEnum;


typedef struct{
  const u8 targetLength;
  u8 numberOfTargets;
  s32 targets[3];
  directionEnum direction;// 0 = down, 1 = up
}plannerHelper;

/*
 * Global helper object to keep track of direction and current targets 
 * {Number of floors, Number of targets(should be zero at start), Targets(also zero at init), Direction}
 */
plannerHelper helper = {3,0,{0},UP};

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
  for(; i < helper.numberOfTargets; i++){
    if(helper.targets[i] == val)
      return 1;
  }
  return 0;
}

void insertTarget(s32 stoppingDistance, s32 currentPosition, s32 target, int (*comparer)(const void *, const void*)){
  s32 minimumTarget = currentPosition;
  if(helper.direction == UP) minimumTarget = currentPosition + stoppingDistance;
  else minimumTarget = currentPosition - stoppingDistance;

  if (comparer(&(target), &(minimumTarget))){
    helper.targets[helper.numberOfTargets] = target;
    helper.numberOfTargets++;
  } else {
    int i;
    for(i = helper.numberOfTargets; i >= 0; i--){
      if(i == 0 || (comparer(&helper.targets[i-1], &target) && !comparer(&helper.targets[i-1], &currentPosition))){
        helper.targets[i] = target;
        helper.numberOfTargets++;
        break;
      }
      else{
        helper.targets[i] = helper.targets[i-1];
      }
    }
  }
}

/* This funCTION CALled when button is pressed to request a particular
   Floor. It has data as a parameter which is a Floor number. */
void requestPosition(s32 target){
  s32 currentPosition = getCarPosition();     //Getting Current Position of Lift
  s32 duty = getMotorCurrentDuty(); // Probably 200 duty per 1cm/s. Check motor.h 
  s32 stoppingDistance = 2 * duty / 200; // duty / 200 => cm's per second. 2 == senconds given to stop
 
  /*
   * function pointers to greater or less than comparisons. 
   * needed to place the requested target in the target array correctly
   */
  int (*cmprFun[2]) (const void*, const void*); 
  cmprFun[DOWN] = cmpGreaterThan;
  cmprFun[UP] = cmpLessThan;

  /*
   * checking if target is already present in the Planner, means
   * if the button pressed has already handled/scheduled to go to the 
   * requested floor.
   */
  if (!existsInPlanner(target))
    insertTarget(stoppingDistance, currentPosition, target,
                 cmprFun[helper.direction]);
}
        
u8 setNextTarget(){
  if (helper.numberOfTargets > 0){
    setCarTargetPosition(helper.targets[0]);
    return 1;
  }
  return 0;
}

/*
 * Removes the first/current target in the array and moves all
 * elements one step forward.
 */
void removeTarget(){
  if (helper.numberOfTargets > 0){
    helper.numberOfTargets--;
    memmove(&(helper.targets), &(helper.targets[1]), sizeof(s32)*helper.targetLength);
    helper.targets[helper.targetLength-1] = 0; 
  }
}

/*
 * Very simple alogrithm for deciding direction. If we are on our way up  
 * we prioritize floors above us. If we are on our way down we prioritize
 * floors below usioritize floors above us. If we are on our way down we prioritize
 * floors below us. We change direction when the next target is in the oposite direction
 * or when we reach top or bottom.
 */
void decideDirection(s32 carPosition){
  if(carPosition > 799 || (helper.numberOfTargets > 0 && helper.targets[0] < carPosition)){
    helper.direction = DOWN;
    sortTargets(cmpGreaterThan);
  }
  if(carPosition < 1  || (helper.numberOfTargets > 0 && helper.targets[0] > carPosition)){
    helper.direction = UP;
    sortTargets(cmpLessThan);
  }
}

volatile floorEvent floorSwitch = ARRIVE;
volatile doorEvent doorSwitch = CLOSED;
xSemaphoreHandle floorSensor;

static void plannerTask(void *params) {
  PinEvent buffer;

  while (1) {
    xQueueReceive(pinEventQueue, &buffer, portMAX_DELAY);

    switch (buffer) {
      case(TO_FLOOR_1) :
        requestPosition(0);
        break;
      case(TO_FLOOR_2) :
        requestPosition(400);
        break;
      case(TO_FLOOR_3) :
        requestPosition(800);
        break;
      case(ARRIVED_AT_FLOOR) :
        floorSwitch = ARRIVE;
        xSemaphoreGive(floorSensor);
        break;
      case(LEFT_FLOOR) :
        floorSwitch = LEAVE; 
        break;
      case(DOORS_CLOSED):
        doorSwitch = CLOSED;
        break;
      case(DOORS_OPENING):
        doorSwitch = OPEN;
        break;
      case(STOP_PRESSED):
        setCarMotorStopped(1);
        break;
      default:
        break;
    }

    /* We dont want to activate a new target if we are at a floor. 
    * That's up to the floorTask to handle
    */
    if (floorSwitch != ARRIVE && doorSwitch == CLOSED) setNextTarget();
  }
}

/*
 * Handles what happen each time we hit a floor.
 * This would typicaly mean that we should remove a target that has ben reached,
 * decide if we want to change direction as well as set the next target
 */
static void floorTask(void *params) {
  portTickType haltTime = 0;
  portTickType currentTime = 0;
  while (1) {
    haltTime = 0;
    //Wait for the a floor to be reached
    xSemaphoreTake(floorSensor, portMAX_DELAY);
    while(floorSwitch == ARRIVE){
      //Remove current target and update direction once we have stopped
      if(getMotorCurrentDuty() == 0 && haltTime == 0){
        removeTarget();
        decideDirection(getCarPosition());
        haltTime = xTaskGetTickCount();
      }
      
      //Tell the elivator to proceed once we have waited for at least 1s at this floor
      currentTime = xTaskGetTickCount();
      if(haltTime != 0 && (currentTime - haltTime) / portTICK_RATE_MS > 1250 &&
          doorSwitch == CLOSED) {
        setNextTarget();
        floorSwitch = LEAVE;
      }
      vTaskDelayUntil(&currentTime, 20 / portTICK_RATE_MS);
    }
  } 
}

void setupPlanner(unsigned portBASE_TYPE uxPriority) {
  floorSensor = xSemaphoreCreateMutex();
  xTaskCreate(plannerTask, "planner", 100, NULL, uxPriority, NULL);
  xTaskCreate(floorTask, "floor", 100, NULL, uxPriority+1, NULL);
}
