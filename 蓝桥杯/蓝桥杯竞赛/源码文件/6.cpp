#include <stdio.h>

int a[10];

int judge(int x)
{
	int m = 0;
	while(x)
	{
		a[m++]=x%10;
		x/=10;
	}
	
	for(int i=m-1;i>0;--i)
	{
		if(a[i]>a[i-1])
		return 0;
	}
	return 1;
}

int main()
{
	int n,m=0;
	
	scanf("%d",&n);
	
	for(int i=1;i<=n;++i)
	{
		if(judge(i))
		m++;
	}
	printf("%d\n",m);
	return 0;
}

