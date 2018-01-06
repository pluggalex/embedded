/**
 * Program skeleton for the course "Programming embedded systems"
 *
 * Lab 1: the elevator control system
 */

/**
 * Class for keeping track of the car position. This is done
 * through pulse rate sensing, e.g., using an optical rotary
 * encoder or by counting pulses along a linear scale in the
 * elevator shaft
 */

#ifndef POSITION_TRACKER_H
#define POSITION_TRACKER_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f10x_gpio.h"

typedef enum {
  Unknown = 0, Up = 1, Down = 2
} Direction;

typedef struct {

  GPIO_TypeDef *gpio;  // Pin to listener at, e.g., GPIOC,
  u16 pin;             // GPIO_Pin_4

  double position;  // The current position in mm. This variable
                    // should only be accessed through
                    // the function "getPosition"

  u8 status;  // used to signal of last pin status

  xSemaphoreHandle lock;  // Mutex semaphore protecting the struct

  portTickType pollingPeriod;  // how often the status of pins is polled

  Direction direction;  // current direction of movement, which
                        // is necessary to know in which direction
                        // to count

} PositionTracker;

/**
 * Setup a tracker object. This creates a task that
 * regularly polls the specified pin for changes
 */
void setupPositionTracker(PositionTracker *tracker,
                          GPIO_TypeDef *gpio, u16 pin,
						  portTickType pollingPeriod,
						  unsigned portBASE_TYPE uxPriority);

/**
 * Set the current direction of movement (Up means that
 * the position is incremented, Down that the position is
 * decremented)
 */
void setDirection(PositionTracker *tracker, Direction dir);

/**
 * Get the current position
 */
double getPosition(PositionTracker *tracker);

/**
 * Get the current direction
 */
Direction getDirection(PositionTracker *tracker);
#endif
