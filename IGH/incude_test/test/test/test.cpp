#include<iostream>
#include<ethercat_manager.h>
#include<stdio.h>
#include <unistd.h>
using namespace ethercat;

int main(void)
{
	EtherCatManager manager();
	while(1)
	{
		sleep(2);
		printf("Func %s Line %d\n", __FUNCTION__, __LINE__);
	}
	return 0;
}

