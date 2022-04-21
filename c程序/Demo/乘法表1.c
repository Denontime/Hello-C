#include<stdio.h>

void main()
{
	int i,j;				//设两个变量控制循环 
	
	for(i=9;i>=0;i--)		//外循环，控制列循环，当内循环一行循环完之后再循环 
	{ 
		for(j=9;j>0;j--)	//内循环，控制行循环 
		{
			if(i>=j) 
			{
				printf("%d*%d=%2d	",i,j,i*j);		//打印字符 	
			}
			else
			{ 
				printf("	");
			} 
		}
		printf("	\n");		//每一次内循环完成后，也就是一行打印完后，换行 
	}	
		 
} 
