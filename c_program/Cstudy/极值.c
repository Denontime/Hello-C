#include<stdio.h>

int a[10];

int max()
{
	int max,m,i;
	max=a[0];	
	for(i=0;i<10;i++)
	{
		if(a[i]>max)
		{
			max=a[i];
			m=i;
		} 
	}
	printf("最大值为%d，其下标为%d\n",max,m);	
} 

int min()
{
	int min,n,i; 
	min=a[0];	
	for(i=0;i<10;i++)
	{
		if(a[i]<min)
		{
			min=a[i];
			n=i;
		}
	}
	printf("最小值为%d，其下标为%d\n",min,n);
}

void main()
{
	int i,n,m;
	printf("请输入十个整数\n");
	
	for(i=0;i<10;i++)
		scanf("%d",&a[i]);
	max();		//调用最大值函数 
	min();		//调用最小值函数 
}

