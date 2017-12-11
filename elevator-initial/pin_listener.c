/**
 * Program skeleton for the course "Programming embedded systems"
 *
 * Lab 1: the elevator control system
 */

/**
 * Functions listening for changes of specified pins
 */

#include "FreeRTOS.h"
#include "task.h"

#include "pin_listener.h"
#include "assert.h"


static void pollPin(PinListener *listener,
                    xQueueHandle pinEventQueue) {

	u8 pin_state = GPIO_ReadInputDataBit(listener->gpio, listener->pin);
	
	if(pin_state == Bit_SET && listener->status == 1) {
		
		xQueueSendToBack(pinEventQueue,&(listener->risingEvent), 20/portTICK_RATE_MS); 
		listener->status = 2;
	}					
	else if(pin_state == Bit_SET && listener->status != 2) listener->status = 1;	
	
  if(pin_state == Bit_RESET && listener->status == 3) {;
		
		xQueueSendToBack(pinEventQueue,&(listener->fallingEvent), 20/portTICK_RATE_MS);
		listener->status = 4;	
	}	
	else if(pin_state == Bit_RESET && listener->status == 2) listener->status = 3;	
}

static void pollPinsTask(void *params) {
  PinListenerSet listeners = *((PinListenerSet*)params);
  portTickType xLastWakeTime;
  int i;

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    for (i = 0; i < listeners.num; ++i)
	  pollPin(listeners.listeners + i, listeners.pinEventQueue);
    
		vTaskDelayUntil(&xLastWakeTime, listeners.pollingPeriod);
  }
}

void setupPinListeners(PinListenerSet *listenerSet) {
  portBASE_TYPE res;

  res = xTaskCreate(pollPinsTask, "pin polling",
                    100, (void*)listenerSet,
					listenerSet->uxPriority, NULL);
  assert(res == pdTRUE);
}
