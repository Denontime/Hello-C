#include <stdio.h>

#define N 10 

void main()
{
	int i, j, t, a[N];				//设两个循环控制变量 i，j ；一个中间变量  t； 一个一维数组a； 

	for(i=0; i<N; i++)				//循环输入数组，之间以  Tab，空格，或回车隔开 
		scanf("%d",&a[i]);			 

	for(i=0; i<N-1; i++)			//第  N-1 次排序 ，
		for(j=0; j<N-1-i; j++)		//遍历数组中的元素， 
		{

			if(a[j] > a[j+1])		//如果前一个大于后一个 
			{
				t = a[j];			//进行交换 
				a[j] = a[j+1];
				a[j+1] = t;
    		}
  		}							//一次排序（内循环）完成后，再进行下一次排序，一共  N-1  次 
	
	printf("\n");					//换行，中间空一行 
	
	for(i=0; i<N; i++)				//循环打印出整个数组 
		printf("%d ",a[i]);
}


