#include <stdio.h>
#define height 10
int helen(int,int);
void main()
{ int l,k,v;
printf("qing shu ru chang he kuan de zhi");
scanf("%d%d",&l,&k);
v=helen(l,k);
printf("chang fang xing de ti ji wei");
printf("%d",v);
}

int helen(int a,int b)
{int c;
	c=a*b*height;
	return c;
}

