#include "sys.h"
#include "usart.h"
#pragma import(__use_no_semihosting)                             
struct __FILE { 
	int handle;
};
FILE __stdout;
_sys_exit(int x) { 
	x = x; 
}
int fputc(int ch, FILE *f) {      
	myputc(ch);
	return ch;
}

