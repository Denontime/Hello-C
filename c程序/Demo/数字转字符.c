#include <stdio.h>
#define uchar unsigned char

static char table[]={'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
 
void num2char(char *str, double number, uchar g, uchar l)
{
    uchar i;
    int temp = number/1;
    double t2 = 0.0;
    for (i = 1; i<=g; i++)
    {
        if (temp==0)
            str[g-i] = table[0];
        else
            str[g-i] = table[temp%10];
        temp = temp/10;
    }
    *(str+g) = '.';
    temp = 0;
    t2 = number;
    for(i=1; i<=l; i++)
    {
        temp = t2*10;
        str[g+i] = table[temp%10];
        t2 = t2*10;
    }
    *(str+g+l+1) = '\0';
}
 
 
 
int main(int argc, char const *argv[])
{
	int i; 
    char str[20];
    num2char(str, 23.56821312, 2, 4);
    for(i=0;i<10;i++)
    {
    	printf("%c\n", str[i]);	
    }
    
    return 0;
}
