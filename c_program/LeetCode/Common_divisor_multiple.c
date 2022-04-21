//
// Created by Denon on 2021/10/24.
//

#include <stdio.h>

int maxNumber(int num1, int num2);

int minNumber(int num1, int num2);

int multiple(int num1, int num2, int divisor);

int EuclideanAlgorithm(int num1, int num2);

int EquivalenceAlgorithm(int num1, int num2);


int main() {
    int n1, n2;
    printf("Please enter the first number:");
    scanf("%d",&n1);
    printf("Please enter the second number:");
    scanf("%d",&n2);
    int div1 = EuclideanAlgorithm(n1, n2);
    int mul1 = multiple(n1, n2, div1);
    printf("EuclideanAlgorithm: divisor %d,multiple %d\n", div1, mul1);
    int div2 = EquivalenceAlgorithm(n1, n2);
    int mul2 = multiple(n1, n2, div2);
    printf("EquivalenceAlgorithm: divisor %d,multiple %d\n", div2, mul2);
    return 0;
}

int EuclideanAlgorithm(int num1, int num2) {    // 辗转相除法（欧几里得法）
    int max = maxNumber(num1, num2);
    int min = minNumber(num1, num2);

    int remainder = max % min;
    while (remainder != 0) {
        max = min;
        min = remainder;
        remainder = max % min;
    }
    return min;
}

int EquivalenceAlgorithm(int num1, int num2) { // 更相减损法
    int count = 0; // 约分次数
    while (num1 % 2 == 0 && num2 % 2 == 0) {
        num1 = num1 / 2;
        num2 = num2 / 2;
        count++;
    }
    int max = maxNumber(num1, num2);
    int min = minNumber(num1, num2);

    int differ = 0;
    while ((differ = max - min) != min) {
        if (differ > min) {
            max = differ;
        } else {
            max = min;
            min = differ;
        }
    }
    if (count != 0) {
        return 2 * count * differ;
    } else {
        return differ;
    }
}

int multiple(int num1, int num2, int divisor) {
    return num1 * num2 / divisor;
}

int maxNumber(int num1, int num2) {
    if (num1 > num2)
        return num1;
    else
        return num2;
}

int minNumber(int num1, int num2) {
    if (num1 < num2)
        return num1;
    else
        return num2;
}
