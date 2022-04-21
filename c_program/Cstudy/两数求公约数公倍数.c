#include <stdio.h>

int main()
{
    printf("hello");
}

int Division(int a, int b)
{
    int max = maxNumber(a, b);
    int min = minNumber(a, b);

    int remainder = max % min;
    while (remainder != 0)
    {
        max = min;
        min = remainder;
        remainder = max % min;
    }
}

int maxNumber(int a, int b)
{
    if (a > b)
        return a;
    else
        return b;
}

int minNumber(int a, int b)
{
    if (a < b)
        return a;
    else
        return b;
}
