/*
	Name: �������� 
	Copyright: 1.0 
	Author: Martin Mar 
	Date: 14/02/20 16:45
	Description: ��������һ������Ϊn�����У���������а���С�����˳�����С�1<=n<=200
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
