#include<stdio.h>
void main()
{
    int i,j,row=0,colum=0,max;		//max�����ֵ��row:�У�colum����   ���Ǳ����������ȡ 
    int a[3][3];		//�����ά���� 
    
    for(i=0;i<3;i++)				// 
    	for(j=0;j<3;j++)			// 
    	scanf("%d",&a[i][j]);		// �������������� 
    
    max=a[0][0];					//�������һ�е�һ����ֵ�������ֵ���� 
    
	for (i=0;i<=2;i++)
    	for (j=0;j<=3;j++)			//�� max�����������е�Ԫ�ذ����Ƚ� 
    	{
    	    if (a[i][j]>max)		//���Ԫ��ֵ����max�е�ֵ 
        	{
            	max=a[i][j];		//����ֵ����max 
            	row=i+1;			//����ǰ�кŸ���row 
            	colum=j+1;			//����ǰ�кŸ���colum 
        	}
    	}
	
	printf("max=%d,row=%d,colum=%d",max,row,colum);		//������ֵ���У��� 
	printf("\n");		//���� 

}
