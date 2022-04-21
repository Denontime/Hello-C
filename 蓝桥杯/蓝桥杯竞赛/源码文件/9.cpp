#include <stdio.h>
#include <stdlib.h>

int m=0;
void qu(int &d)
{
	if(d>10000)
		d-=10000;
}

void re(int a,int b)
{
	int z = abs(a-b);
	m++;
	qu(m);
	for(int i=1;i<z;++i)
	{
		re(b,i);
	}
}

int main()
{
	int n;
	scanf("%d",&n);
	
	for(int i=1;i<=n;++i)
		re(4,i);
		
	printf("%d\n",m);
	
	return 0;
}

