/* Note:Your choice is C IDE */
#include "stdio.h"
int alice(int,int);
void main()
{
	int a,b,s;
	printf("qing shu ru a he b de zhi:");
	scanf("%d%d",&a,&b);
	s = alice(a,b);
	printf("s de zhi wei :");
	printf("%d",s);
} 
int alice(int x,int y)
{ 
	int result;
	result = x + y;
	return result;
}

