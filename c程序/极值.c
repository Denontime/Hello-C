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
	printf("���ֵΪ%d�����±�Ϊ%d\n",max,m);	
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
	printf("��СֵΪ%d�����±�Ϊ%d\n",min,n);
}

void main()
{
	int i,n,m;
	printf("������ʮ������\n");
	
	for(i=0;i<10;i++)
		scanf("%d",&a[i]);
	max();		//�������ֵ���� 
	min();		//������Сֵ���� 
}

