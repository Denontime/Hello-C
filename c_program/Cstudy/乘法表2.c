/* Note:Your choice is C IDE */
#include "stdio.h"
void main()
{
    int i,j;
    
    for(i=9;i>=0;i--)
    {
    for(j=1;j<=9;j++)
    {
    		if(i>=j) 
			{
				printf("%d*%d=%2d	",i,j,i*j);		//´òÓ¡×Ö·û 	
			}
			else
			{ 
				printf("	");
			}
    }
    printf("\n");
    }
}