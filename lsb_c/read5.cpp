#include <stdio.h>
#include <string>
#include <windows.h>

int main()
{
FILE *in;
unsigned int i;
char FName [90],ch;
printf ("请输入加密后位图文件的文件名: \n");
scanf ("%s", FName) ;
if ((in = fopen (FName,"rb"))==NULL)
{
printf ("无法打开文件。\n");
return 1;
}
fseek(in,54L,0);
printf ("解密后的文字: \n");
do{
ch = 0;
for(i=0;i<8;i++)
ch+=(fgetc(in) & 0x01)<<i;



putchar(ch); //把每8个字节的末位信息组合成一个字符,然后将其输出。
}while(ch);
putchar('\n');
return 0;
}
