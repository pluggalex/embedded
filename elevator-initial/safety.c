/**
 * Program skeleton for the course "Programming embedded systems"
 *
 * Lab 1: the elevator control system
	 
   Updates by Group 5:  Added a Function 'isInRange'. 
												Updated Safety Task to meet the Environment & 
												Safety Requirements.
 */

/**
 * This file defines the safety module, which observes the running
 * elevator system and is able to stop the elevator in critical
 * situations
 */

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "global.h"
#include "assert.h"

#define POLL_TIME (10 / portTICK_RATE_MS)

#define MOTOR_UPWARD   (TIM3->CCR1)
#define MOTOR_DOWNWARD (TIM3->CCR2)
#define MOTOR_STOPPED  (!MOTOR_UPWARD && !MOTOR_DOWNWARD)

#define STOP_PRESSED  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)
#define AT_FLOOR      GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)
#define DOORS_CLOSED  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)

static portTickType xLastWakeTime;

static void check(u8 assertion, char *name) {
  if (!assertion) {
    printf("SAFETY REQUIREMENT %s VIOLATED: STOPPING ELEVATOR\n", name);
    for (;;) {
      setCarMotorStopped(1);
      vTaskDelayUntil(&xLastWakeTime, POLL_TIME);
    }
  }
}

/* Function added to compare two values and return a boolean if the value is 
	 in less than equal to the range.
*/
bool isInRange(double value, double compare, double range){
  return fabs(value-compare) <= range;
}

/* Safety Task which will take care, if the system meets all the requirements */
static void safetyTask(void *params) {
  s16 timeSinceLastCheck;
  double distance;
  bool direction = 1;
  bool stopActive = 0;
  s16 timeSinceStopPressed = -1;
  s16 timeSinceStopedAtFloor = 1000;  // time in ms
  double lastKnownPosition = getCarPosition();
  double currentPosition = 0 ;
  s32 lastKnownAcceleration = 0; //cm's / second
  s16 tickLastCheck = xTaskGetTickCount();
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
		currentPosition = getCarPosition();
    // Environment assumption 1: the doors can only be opened if
    //                           the elevator is at a floor and
    //                           the motor is not active
    check((AT_FLOOR && MOTOR_STOPPED) || DOORS_CLOSED, "env1");


    // Environment assumption 2: The elevator moves at a 
		//													 maximum speed of 50cm/s	
		timeSinceLastCheck = (xTaskGetTickCount() - tickLastCheck) / portTICK_RATE_MS; //time in ms
    distance = currentPosition - lastKnownPosition; //distance in cm
    tickLastCheck = xTaskGetTickCount(); //remember current tick for next iteration
    check(!(timeSinceLastCheck > 0 && distance/timeSinceLastCheck >= 0.5), "env2");


    // Environment assumption 3 : If the ground floor is put at 0cm 
		//														in an absolute coordinate system, the second floor 
		//														is at 400cm and the third floor at 
		//														800cm (the at-floor sensor reports a floor with 
		//														a threshold of +-0.5cm)
    check(!(AT_FLOOR && !(isInRange(lastKnownPosition, 0, 0.5) || isInRange(lastKnownPosition, 400, 0.5) || isInRange(lastKnownPosition, 800, 0.5))), "env3");
    lastKnownPosition = currentPosition;  // get the last known position of
                                          // lift.

    // Environment assumption 4: We make sure thate the input from the motor
    // seems sane.
    // If the motor indicates UPWARD and DOWNWARD something is very wrong and we
    // should stop
    check(!(MOTOR_UPWARD && MOTOR_DOWNWARD), "env 4 - motor input is bonkers!");

    /* System requirement 1: if the stop button is pressed, the motor is
    //                       stopped within 1s
    */
    if (STOP_PRESSED) {
      if (timeSinceStopPressed < 0)
        timeSinceStopPressed = 0;
      else
        timeSinceStopPressed += POLL_TIME;

      check(timeSinceStopPressed * portTICK_RATE_MS <= 1000 || MOTOR_STOPPED,
            "req1");
    } else {
      timeSinceStopPressed = -1;
    }

    /* System requirement 2: the motor signals for upwards and downwards
                           movement are not active at the same time
    */
    check(!MOTOR_UPWARD || !MOTOR_DOWNWARD, "req2");

    /* Safety requirement 3: The elevator may not pass the end positions, 
		  											 that is, go through the roof or the floor
		
		   End Positions: Lowest Bound - 0 cms
					  				  Upper Bound - 800 cms (Assuming till 3rd Floor) 
    */
    check((currentPosition <= 800 && currentPosition >=0), "req3");

    /* Safety requirement 4
		   If Lift is neither moving UPWARD not moving DOWNWARD 
			 -- Then someone pressed the Stop Button or If Lift is 
					at the Floor	
		*/
    if(STOP_PRESSED) stopActive = 1;
    if(lastKnownAcceleration > (getMotorCurrentDuty() / 200) && !STOP_PRESSED) stopActive = 0;
    if (MOTOR_STOPPED && lastKnownAcceleration > 0) check(stopActive || AT_FLOOR, "req4");

    /* Safety requirement 5
     * The left should wait for at least 1 second in stand still 
     * when a target floor is reached.
     */
    if(AT_FLOOR && MOTOR_STOPPED){
      if(timeSinceStopedAtFloor < 0) timeSinceStopedAtFloor = 0;
      else timeSinceStopedAtFloor += timeSinceLastCheck;
    }
    else if(!MOTOR_STOPPED && timeSinceStopedAtFloor >= 0){
      check(timeSinceStopedAtFloor >= 1000, "req5");
      timeSinceStopedAtFloor = -1;
    }
   
    /* Safety requirement 6: Elevator should not change the direction of motion , 
		 * Elevator is allowed to change the directions only at Floors.
     */
    if(!AT_FLOOR) check(direction == (MOTOR_UPWARD > MOTOR_DOWNWARD), "req6");
    else direction = (MOTOR_UPWARD > MOTOR_DOWNWARD); 
    
    lastKnownAcceleration = getMotorCurrentDuty() / 200;
    vTaskDelayUntil(&xLastWakeTime, POLL_TIME);
  }
}
void setupSafety(unsigned portBASE_TYPE uxPriority) {
  xTaskCreate(safetyTask, "safety", 100, NULL, uxPriority, NULL);
}
