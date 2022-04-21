#include<bits/stdc++.h>
using namespace std;

int main(){
	int n=2019,ans=0,f,j;
	for(int i=9;i<=n;++i){
		f=0;
        j=i;
		while(j){
			if(j%10==9){
				f=1;
				break;
			}
			j/=10;
		}
		if(f)ans++;
	}
	printf("%d",ans);//544
}

