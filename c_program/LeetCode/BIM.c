#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err34-c"

#include<stdio.h>

void main() {
    float weight, high, bim;
    printf("���������������:");
    scanf("%f%f", &weight, &high);
    bim = weight / (high * high);
    printf("����BIMֵΪ:%f.\n", bim);

    if (bim < 18.5)
        printf("����BIMֵС��18.5������.");
    else if (bim >= 18.5 && bim < 24.9)
        printf("����BIMֵ����18.5С��24.9����.");
    else if (bim >= 24.5 && bim < 27.9)
        printf("����BIMֵ����25.0С��27.9����.");
    else
        printf("����BIMֵ����27.9����.");
}

#pragma clang diagnostic pop