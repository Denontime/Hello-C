#include<stdio.h>
void main()
{
    int i,j,row=0,colum=0,max;		//max：最大值，row:行，colum：列   都是变量名，随便取 
    int a[3][3];		//定义二维数组 
    
    for(i=0;i<3;i++)				// 
    	for(j=0;j<3;j++)			// 
    	scanf("%d",&a[i][j]);		// 向数组输入数据 
    
    max=a[0][0];					//将数组第一行第一列数值赋给最大值变量 
    
	for (i=0;i<=2;i++)
    	for (j=0;j<=3;j++)			//将 max变量与数组中的元素挨个比较 
    	{
    	    if (a[i][j]>max)		//如果元素值大于max中的值 
        	{
            	max=a[i][j];		//将该值赋给max 
            	row=i+1;			//将当前行号赋给row 
            	colum=j+1;			//将当前列号赋给colum 
        	}
    	}
	
	printf("max=%d,row=%d,colum=%d",max,row,colum);		//输出最大值，行，列 
	printf("\n");		//换行 

}
