#include<stdio.h>
#include<Windows.h>

#define N 170 
#define T 20 
void main()
{
	double x,y;
	
	while(1)
	{
	
		for(x=0;x<N;x++)
		{
			for(y=0;y<x;y++)
			{
				printf(" ");
			}
			printf("*");
			printf("\n");
			Sleep(T);
		}	
	
	
			for(x=N;x>0;x--)
			{
				for(y=x;y>0;y--)
				{
					printf(" ");
				}
				printf("*");
				printf("\n");
				Sleep(T);			
			}	
		
	}

}



