


/*头文件*/
#include <reg52.h>      
#include <intrins.h>
#define uint unsigned int
#define uchar unsigned char

sbit P01 = P1^0;  /*定义P1.1端口*/

void main(void)
{
        P02 = 0;
        P03 = 0;
        p04 = 0;
        p05 = 0;
	P01 = 0; /*点亮与P0.0的LED发光二极管*/
}

