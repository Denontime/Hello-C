/*
	Name: 圆的面积 
	Copyright: 1.0 
	Author: Martin Mar 
	Date: 13/02/20 23:12
	Description: 输出一行，包含一个实数，四舍五入保留小数点后7位，表示圆的面积。
*/

#include <stdio.h>

#define pi 3.14159265358979323

int main()
{
	int r;
	double area;
	
	scanf("%d",&r);
	
		area=r*r*pi;
	
		printf("%.7lf",area);
	
	return 0;
} 
