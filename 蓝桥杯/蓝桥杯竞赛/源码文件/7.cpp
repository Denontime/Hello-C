#include<stdio.h>
#include<string.h>

char s[100];
int i=0;

/**
 * @brief �ж��ַ��Ƿ�ΪԪ����ĸ a, e, i, o, u
*/
int ch(char c)
{
	return c=='a'||c=='e'||c=='i'||c=='o'||c=='u';
}

/**
 * @brief �жϸ����εĸ���
*/
int fu()
{
	int ans=0;
	for(;i<strlen(s);++i)
	{
		if(ch(s[i])==false)
			ans++; 
		else 
			break;
	} 
	return ans;
}

/**
 * @brief �ж�Ԫ���εĸ���
*/
int yuan()
{
	int ans=0;
	for(;i<strlen(s);++i)
	{
		if(ch(s[i])==true)
			ans++; 
		else 
			break;
	}
	return ans;
}

int main()
{
	scanf("%s",&s);
	if(fu()>0 && yuan()>0 && fu()>0 && yuan()>0 && i==strlen(s))
	printf("yes\n");
	else 
	printf("no\n");//��·����
	
	return 0;
	
}

