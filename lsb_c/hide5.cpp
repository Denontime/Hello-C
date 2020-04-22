#include "stdio.h"
#include "string.h"
#include "malloc.h"

int main ( )
{
FILE *in, *out;
unsigned int i, j,m, n;
char inFileName [90], outFileName[90], *pch, ch[1000], *temp;
printf ("请输入原始位图文件的文件名: \n");
scanf ("%s", inFileName);
printf ("请输入加密程序产生的新位图文件的文件名: \n");
scanf ("%s", outFileName);
fflush (stdin);
printf ("请输入你要保密的文字: \n");
pch = ch;
gets(pch);
for(m=0, temp=( char* )malloc( 8*strlen(ch)); m<strlen(ch); pch++,m++)
for (n=0; n<8; n++)
temp [ 8*m + n] = 0x01 & *pch>>n;//以上嵌套循环体的作用是把输入的文字信息按位离散到一个动态数组中。
if ( ( in = fopen ( inFileName,"rb") ) ==NULL)
{
printf ("无法打开原始位图文件。\n") ;
return 1;
}
if ( (out = fopen (outFileName,"wb") )==NULL)
{
printf ("无法打开加密位图文件。\n") ;
return 2;
}
for ( i=1, j=0; !feof(in) ; i++)
{
if ( i<=54)
fputc ( fgetc (in) , out) ; //位图文件的前54个字节保持原样不变保存到新文件中
else
{
if ( j<8*strlen(ch) )
fputc ((fgetc(in) & 0xfe) + temp[j],out) ;//把密文数据逐个附加到该位图文件“实际位图数据”部分的每一个字节的末位
else
fputc (fgetc ( in) & 0xfe, out);
j++;
}
}
fclose (in);
fclose (out);
return 0;
}
