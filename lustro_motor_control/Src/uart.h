#include "stm32f1xx.h"
#include <string.h>

UART_HandleTypeDef uart;



void uart_send_string(char* s)
{
 HAL_UART_Transmit(&uart, (uint8_t*)s, strlen(s), 1000);
}

void send_char(char c)
{
 HAL_UART_Transmit(&uart, (uint8_t*)&c, 1, 1000);
}

int __io_putchar(int ch)
{
 if (ch == '\n')
 send_char('\r');
 send_char(ch);
 return ch;
}


 

 

