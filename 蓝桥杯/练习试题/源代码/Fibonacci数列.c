/*
	Name: Fibonacci数列
	Copyright: 1.0 
	Author: Martin Mar 
	Date: 13/02/20 23:12
	Description: Fibonacci数列的递推公式为：Fn=Fn-1+Fn-2，其中F1=F2=1
				当n比较大时，Fn也非常大，现在我们想知道，Fn除以10007的余数是多少。
*/
 
#include <stdio.h>

int main()
{
	long int n,fib,fib_1=1,fib_2=1,temp,i;
	
	scanf("%ld",&n);
	
	if(n<3)
	{
		printf("1");
	}
	else
	{
		for(i=0;i<n-2;i++)
		{
			temp=fib_1+fib_2;
			fib=temp%10007;
			fib_1=fib_2%10007;
			fib_2=fib;
			
		}

		printf("%ld",fib);
	
	}
	
	return 0;
}


