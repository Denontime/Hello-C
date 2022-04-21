#include <cstring>
#include <iostream>
#include <string>
using namespace std;

int main()
{
    {
        string str = "2000,1587,1730,2000,2000,P";
        char string[40] = {'0'};
        //char test[10];
        char *token;
        const char s[2] = ",";

        //strncpy(test, str.c_str(), 4);
        strcpy(string, str.c_str());
        token = strtok(string, s);
        printf("%s\n", token);

        cout << endl;
    }

    {
        string s = "12345";
        char test[10];
        int i = 0;
        for (i = 0; i < 10; i++)
        {
            test[i] = 'z';
        }

        strncpy(test, s.data(), 6);
        for (i = 0; i < 10; i++)
        {
            printf("%c ", test[i]);
        }

        cout << endl;
    }

    return 0;
}
