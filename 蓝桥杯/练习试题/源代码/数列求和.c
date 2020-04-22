/*
	Name: 数列排序 
	Copyright: 1.0 
	Author: Martin Mar 
	Date: 14/02/20 16:45
	Description: 　　给定一个长度为n的数列，将这个数列按从小到大的顺序排列。1<=n<=200
*/

#include<stdio.h>

int main()
{
	int n,i,j,tmp,a[200];
	
	scanf("%d",&n);
	
	for(i=0;i<n;i++)
		scanf("%d",&a[i]);
		
	for(i=0;i<n;i++)
	{
		for(j=0;j<n-1-i;j++)
		{
			if(a[j]>a[j+1])
			{
				tmp=a[j];
				a[j]=a[j+1];
				a[j+1]=tmp;	
			}
			else 
			continue;
		}
	}
	
	for(i=0;i<n;i++)
		printf("%d ",a[i]);

	return 0;
} 
