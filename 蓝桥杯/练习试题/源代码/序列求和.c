/*
	Name: ������� 
	Copyright: 1.0 
	Author: Martin Mar 
	Date: 13/02/20 23:12
	Description: ��1+2+3+...+n��ֵ---1 <= n <= 1,000,000,000
*/

#include<stdio.h>

int main()
{
	long long n,sum;
	
	scanf("%lld",&n);
	
	sum = n+(n*(n-1))/2;
	
	printf("%lld",sum);
	
	return 0;
} 
