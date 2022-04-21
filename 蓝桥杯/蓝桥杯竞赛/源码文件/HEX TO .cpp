#include <iostream>
#include <string>
#include <sstream>

using namespace std;

int main ()
{ 
	string hex;
	while (getline(cin,hex))
	{
		int a;
		stringstream ch(hex);
		while(ch >> a)
		{
			while(a == '\0')
			{
				if(a>=0 && a<=9)
				{
					cout<<"number"<<endl;
				}
				else
				{
					cout<<"char"<<endl;
				}
			}
		}
	} 

    return 0;
}
