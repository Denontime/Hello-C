#include <stdio.h>				//头文件调用	

double max;						//定义全局变量 max 
double a[10];					//定义全局数组， 用来存放输入的数 

void array_max(int n)			//求最高分的函数，函数有一个参数 n，决定比较几次 
{	
	int i;
	max=a[0];					//将第一个数的值赋给max 
	for(i=1;i<n;i++)		//进行循环，控制 max与数组内所有元素比较 
		if(max<a[i])			//如果数组中元素的大于 max中的，就将该值赋给 max 
			max=a[i];			//将数组内所有元素比较完，max里面就是最大的 
}

void main()						//主函数 
{	
	int i;
	for(i=0;i<10;i++)		//控制 scanf 输入 
		scanf("%lf",&a[i]);		//输入 10个数 
		
	array_max(10);				//调用比较函数，比较数组中的数据 
	
	printf("最高分为：%.2lf.\n",max);		//打印 max 变量 
}

/*			注意：1、数组也是一种类型的变量 
				  2、全局变量：如 max，a[10];在整个程序的任何地方都可以调用 
				     局部变量：如 i，只能在定义该变量的函数内使用
			比如，在 array_max中定义一个变量 y，你想在 main 里调用是不可以的 
				  3、函数调用的形式：  函数名（参数 ）； 
				  参数会传递到被调用的函数中，例如 main中的 array_max(10); 
			这个 10就会传递给定义函数时的  n  上，在 array_max函数内可以使用    */          
