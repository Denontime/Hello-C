/*
	Name: Fibonacci����
	Copyright: 1.0 
	Author: Martin Mar 
	Date: 13/02/20 23:12
	Description: Fibonacci���еĵ��ƹ�ʽΪ��Fn=Fn-1+Fn-2������F1=F2=1
				��n�Ƚϴ�ʱ��FnҲ�ǳ�������������֪����Fn����10007�������Ƕ��١�
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


