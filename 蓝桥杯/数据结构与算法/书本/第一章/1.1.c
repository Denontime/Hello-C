/*
	Name: 1.1 
	Copyright: 1.0 
	Author: Martin Mar 
	Date: 11/02/20 20:33
	Description: �ж����������Ƿ�Ϊ���� 
*/

#include <stdio.h>

int main()
{
	int num,tan,i,j=0;
	
	printf("Plese enter a number:");
	scanf("%d",&num);
	
	for(i=1;i<=num;i++)
	{
		tan = num % i;
		
		if(tan==0)
		{
			j++;
		}
	}
	
	if(j<=2)
	{
		printf("This is a prime  number!");
	}
	else
	{
		printf("This is't a prime  number!");
	}
	
	return 0;
} 
