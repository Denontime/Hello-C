#include "stdio.h"
#include "string.h"

void main()
{
	int len; //????len???????????????????????????
	char str1[20] = "Chinese", str2[20] = "China", str3[30];
	//???????????????str1,str2,str3;????str1,str2?????

	len = strlen(str1);					 //??? str1 ???????????????????????? len
	printf("len=%d,str1=%s", len, str1); //??? len???????? str1?????????

	strcpy(str3, str1); //??????? str1???????????????? str3??
	puts(str3);			//????????????????? z str3???????????

	if (strcmp(str1, str2) > 0) //???????????????? str1?? str2????? str1>str2
		puts(str1);				//??? str1
	else						//????
		puts(str2);				//??? str2

	strcat(str1, str2); //????????? str2????? str1??
	puts(str1);			//??? str1

	/*	gets(??????);	??????????? ????????????????????????????	
		gets(??????);	???????????? ??????????????????????????? 
		strlen(??????);		??????????????????????????????????	
		strcpy(??????1????????2); 	????????2???????????????????1???	?? -?????->? 
		strcat(??????1????????2)??	????????2???????????  ?????  ??????1??????????? ???? ????????????1?? 
		strcmp(??????1????????2)??	??????????????????????????????ASIIC?????????????
		????????? 0??		?????? ??????  1??		???????? ???? -1 ??		*/
}
