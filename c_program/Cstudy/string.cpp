#include <cstring>
#include <iostream>
#include <string>
using namespace std;

int main()
{
   string str = "2000,1587,1730,2000,2000,P";
   const char s[2] = ",";
   char *token;

   /* 获取第一个子字符串 */
   token = strtok(str.data(), s);

   /* 继续获取其他的子字符串 */
   while (token != NULL)
   {
      printf("%s\n", token);

      token = strtok(NULL, s);
   }

   return (0);
}
