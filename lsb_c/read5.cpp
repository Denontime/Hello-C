#include <stdio.h>
#include <string>
#include <windows.h>

int main()
{
FILE *in;
unsigned int i;
char FName [90],ch;
printf ("��������ܺ�λͼ�ļ����ļ���: \n");
scanf ("%s", FName) ;
if ((in = fopen (FName,"rb"))==NULL)
{
printf ("�޷����ļ���\n");
return 1;
}
fseek(in,54L,0);
printf ("���ܺ������: \n");
do{
ch = 0;
for(i=0;i<8;i++)
ch+=(fgetc(in) & 0x01)<<i;



putchar(ch); //��ÿ8���ֽڵ�ĩλ��Ϣ��ϳ�һ���ַ�,Ȼ���������
}while(ch);
putchar('\n');
return 0;
}
