#include <stdarg.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "utils/uartstdio.h"

extern SemaphoreHandle_t xUARTMutex;

void UART_Print(const char *format, ...){
    va_list args;
    va_start(args, format);

    xSemaphoreTake(xUARTMutex, portMAX_DELAY);
    UARTvprintf(format, args);
    xSemaphoreGive(xUARTMutex);

    va_end(args);
}
