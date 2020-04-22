/*
	Name: 1.2
	Copyright: 1.0
	Author: Martin Mar
	Date: 11/02/20 20:52
	Description: º∆À„£∫s=1+1/2!+1/3!+°≠°≠+1/n! 
*/

#include <stdio.h>

int fac(int f)
{
	int result,i;
	
	for(i=1;i<=f;i++)
	{
		result = result * i;
	}
	
	return (result);
}

int main()
{
	int num,i;
	float sum=0.0;
	
	printf("º∆À„£∫s=1+1/2!+1/3!+°≠°≠+1/n!  \n");
	printf("Please enter n:");
	scanf("%d",&num);
	
	for(i=1;i<=num;i++)
	{
		sum = sum + (1 / fac(i));
	}
	
	printf("The result : %d",fac(num));
	return 0;
}


