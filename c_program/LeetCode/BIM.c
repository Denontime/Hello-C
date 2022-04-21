#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err34-c"

#include<stdio.h>

void main() {
    float weight, high, bim;
    printf("请输入体重与身高:");
    scanf("%f%f", &weight, &high);
    bim = weight / (high * high);
    printf("此人BIM值为:%f.\n", bim);

    if (bim < 18.5)
        printf("此人BIM值小于18.5较消瘦.");
    else if (bim >= 18.5 && bim < 24.9)
        printf("此人BIM值大于18.5小于24.9正常.");
    else if (bim >= 24.5 && bim < 27.9)
        printf("此人BIM值大于25.0小于27.9超重.");
    else
        printf("此人BIM值大于27.9肥胖.");
}

#pragma clang diagnostic pop