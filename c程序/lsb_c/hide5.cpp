#include "stdio.h"
#include "string.h"
#include "malloc.h"

int main ( )
{
FILE *in, *out;
unsigned int i, j,m, n;
char inFileName [90], outFileName[90], *pch, ch[1000], *temp;
printf ("������ԭʼλͼ�ļ����ļ���: \n");
scanf ("%s", inFileName);
printf ("��������ܳ����������λͼ�ļ����ļ���: \n");
scanf ("%s", outFileName);
fflush (stdin);
printf ("��������Ҫ���ܵ�����: \n");
pch = ch;
gets(pch);
for(m=0, temp=( char* )malloc( 8*strlen(ch)); m<strlen(ch); pch++,m++)
for (n=0; n<8; n++)
temp [ 8*m + n] = 0x01 & *pch>>n;//����Ƕ��ѭ����������ǰ������������Ϣ��λ��ɢ��һ����̬�����С�
if ( ( in = fopen ( inFileName,"rb") ) ==NULL)
{
printf ("�޷���ԭʼλͼ�ļ���\n") ;
return 1;
}
if ( (out = fopen (outFileName,"wb") )==NULL)
{
printf ("�޷��򿪼���λͼ�ļ���\n") ;
return 2;
}
for ( i=1, j=0; !feof(in) ; i++)
{
if ( i<=54)
fputc ( fgetc (in) , out) ; //λͼ�ļ���ǰ54���ֽڱ���ԭ�����䱣�浽���ļ���
else
{
if ( j<8*strlen(ch) )
fputc ((fgetc(in) & 0xfe) + temp[j],out) ;//����������������ӵ���λͼ�ļ���ʵ��λͼ���ݡ����ֵ�ÿһ���ֽڵ�ĩλ
else
fputc (fgetc ( in) & 0xfe, out);
j++;
}
}
fclose (in);
fclose (out);
return 0;
}
