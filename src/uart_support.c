
#include "uart_support.h"

static UartDev_IOB *conio;

/* Initialize serial console */
void conio_init(uint32_t baudrate)
{
	//int32_t ret = SUCCESS;
	
	ATTACH_MFS();
	
	conio = &UartDev[UART_MFS];
	conio->Cfg.BitOrder = UartDev_BITORDER_LSB;
	conio->Cfg.BufSize  = UART_BUFSIZE;
	conio->Cfg.BaudRate = baudrate;

	//ret = conio->Init();
	conio->Init();
	
	return;
}

int32_t conio_available(void)
{
	return conio->RxAvailable();
	//return 0;
}

float get_float_input(void)
{
	uint8_t data[64];
	
	cgets((char *)data, sizeof(data));
	
	return atof((const char *)data);
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Send 1 character */
inline void putch(uint8_t data)
{
	/* Interrupt Version */
	/*
	while(!USART_TXBuffer_FreeSpace(pUSART_Buf));
	USART_TXBuffer_PutByte(pUSART_Buf,data);
	*/
	uint8_t txdata = data;
	int32_t size = 1;
	
	while(1){
		conio->BufTx(&txdata, &size, UartDev_FLAG_NONBLOCKING);
		if(size == 1) break;
		size = 1;
	}
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Receive 1 character */
uint8_t getch(void)
{
	/*
	if (USART_RXBufferData_Available(pUSART_Buf))  return USART_RXBuffer_GetByte(pUSART_Buf);
	else										   return false;
	*/
	uint8_t data;
	int32_t size = 1;
	conio->BufRx( &data, &size, UartDev_FLAG_NONBLOCKING);
	
	if(size == 1) return data;
	else return false;
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Return 1 if key pressed */
uint8_t keypressed(void)
{
  return 0; //‚»‚Ì‚¤‚¿ŽÀ‘•‚µ‚½‚¢
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Send a string */
void cputs(char *s)
{
  while (*s)
    putch(*s++);
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Receive a string, with rudimentary line editing */
void cgets(char *s, int bufsize)
{
  char *p;
  int c;

  memset(s, 0, bufsize);

  p = s;

  for (p = s; p < s + bufsize-1;)
  {
    /* 20090521Nemui */
	do{		
		c = getch();
	}while(c == false);
	/* 20090521Nemui */
    switch (c)
    {
      case '\r' :
      case '\n' :
        putch('\r');
        putch('\n');
        *p = '\n';
        return;

      case '\b' :
        if (p > s)
        {
          *p-- = 0;
          putch('\b');
          putch(' ');
          putch('\b');
        }
        break;

      default :
        putch(c);
        *p++ = c;
        break;
    }
  }

  return;
}
