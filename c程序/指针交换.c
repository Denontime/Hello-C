#include <stdio.h>

void main() 
{
	int a,b,*m,*n;
	
	printf("Input two number:");
	
	scanf("%d%d",&a,&b);
	
	if(a>b)
	{
		m = &a;
		n = &b;	
	}	
		
	else
	{ 
		m = &b;
		n = &a;
	}
	 
	printf("%d , %d",*m,*n);
		
}
