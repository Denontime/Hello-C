#include "stdio.h" 
#include "string.h"

void main()
{
	int len;				//定义len变量，用来储存求得的字符串长度 
	char str1[20] = "Chinese",str2[20] = "China",str3[30];		
							//定义字符型一维数组str1,str2,str3;并给str1,str2赋初值 
	
	
	len = strlen(str1);		//求出 str1 内储存的字符串的长度，将值赋给 len 
	printf("len=%d,str1=%s",len,str1);		//打印 len的值，以及 str1内储存的字符 
	
	strcpy(str3,str1);		//将字符串 str1内储存的字符串复制到 str3内 
	puts(str3);				//用字符串打印函数打印  str3内储存的字符串 
	
	if(strcmp(str1,str2)>0)	//用字符串比较函数比较 str1与 str2，如果 str1>str2 
		puts(str1);			//打印 str1 
	else					//否则 
		puts(str2);			//打印 str2 
		
	strcat(str1,str2);		//用字符串将 str2连接到 str1后 
	puts(str1);				//打印 str1 
	
	/*	gets(数组名);	字符串输入函数 ，将输入的字符赋给数组，以回车结束	
		gets(数组名);	字符串输出函数 ，将数组内储存的字符串打印到屏幕 
		strlen(数组名);		求数组内储存的字符串的长度，返回一个整数	
		strcpy(数组名1，数组名2); 	将数组名2内的字符串复制到数组名1内，	后 -复制到->前 
		strcat(数组名1，数组名2)；	将数组名2内储存的字符串  连接到  数组名1内储存的字符串 后面 ，储存在数组1中 
		strcmp(数组名1，数组名2)；	将两个数组内储存的字符串自左至右按ASIIC码的值比较，如果：
		相同，返回 0；		前大于后 ，返回  1；		后大于前， 返回 -1 ；		*/ 
}
