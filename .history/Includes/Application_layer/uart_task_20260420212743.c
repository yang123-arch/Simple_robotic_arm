#include "uart_task.h"
#include "uart_driver.h"

#include "FreeRTOS.h"
#include "task.h"

#include "freertos_handle.h"

void

    void
    UART_Task(void *argument) {

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}
