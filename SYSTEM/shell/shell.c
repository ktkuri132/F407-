#include <stm32f4xx.h>

#include "stdio.h"
#include <usart.h>
#include "string.h"

uint16_t shscanf(char *buff)
{
    while(!(USART_RX_STA&0xC000));
    strcpy(buff,(char*)USART_RX_BUF);
    uint16_t size = USART_RX_STA&0x3FFF;
    USART_RX_STA=0;
    return size;

}

void shell_terminal()
{
    char command[100];
    
    while (1)
    {
        printf("Shell> ");
        shscanf(command);

        
        // Process the command
        
        // Example: Print the command
        printf("Command: %s", command);
    }
}

