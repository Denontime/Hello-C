#include<bits/stdc++.h>
using namespace std;

int main(){
	int ans = 0 ,i,n=1200000;
	for( i=1;i*i<n;++i){
		if(n%i==0)ans+=2;
	}
	if(i*i==n)ans+=1;
	printf("%d",ans);//96
}

