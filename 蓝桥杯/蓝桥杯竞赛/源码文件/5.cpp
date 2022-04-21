#include <stdio.h>

int main()
{
	int n,i,j,k,m=0;
	int a[10000]={};
	
	scanf("%d",&n);
	
	for(i=0;i<n;i++)
	{
		scanf("%d",&a[i]);
	}
	
	for(i=0;i<n-2;i++)
	{
		for(j=i+1;j<n-1;j++)
			{
				for(k=j+1;k<n;k++)
					{
						if(a[i]<a[j] && a[j]<a[k])
						{	
							m++;
							a[j]=0;
							break;
						}
					}
			}
	}
	
	printf("%d",m);
	
	return 0;
}
