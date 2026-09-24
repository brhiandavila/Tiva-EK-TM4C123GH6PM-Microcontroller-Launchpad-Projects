#ifndef CMD_TASK_H
#define CMD_TASK_H

#include <stdbool.h>

extern volatile bool bLogPaused;

void CMD_UART0RxInit(void);
void vCommandTask(void *pvParameters);
void UART0IntHandler(void);

#endif // CMD_TASK_H
