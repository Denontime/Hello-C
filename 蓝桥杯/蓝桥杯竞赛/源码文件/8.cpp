#include<bits/stdc++.h>
using namespace std;
const int N = 1e3+3;
int n,m,k;
char g[N][N];
int pos[4][2]={{1,0},{-1,0},{0,1},{0,-1}};//移动方向
struct Node
{
	int x,y;
	Node(int x,int y):x(x),y(y){}
	Node(){}
};

queue<Node> pre;//初始访问
queue<Node> _next;//下次访问

void bfs(){
	int xx,yy;
	Node node; 
	for(int i=0;i<k;++i){
		
		while(!pre.empty()){
			node=pre.front();
			pre.pop();
			for(int j=0;j<4;++j){
				xx = node.x+pos[j][0];
				yy = node.y+pos[j][1];
				if(xx>=0&&xx<n&&yy>=0&&yy<=m&&g[xx][yy]=='.'){
					g[xx][yy]='g';
					_next.push(Node(xx,yy));
				}
			}
		}
		while(!_next.empty()){
			pre.push(_next.front());
			_next.pop();
		}
	}
}

int main(){
	std::ios::sync_with_stdio(false);
	cin.tie(0);//关闭输入同步流
	cin>>n>>m;
	for(int i=0;i<n;++i){
		for(int j=0;j<m;++j){
			cin>>g[i][j];
			if(g[i][j]=='g')pre.push(Node(i,j));
		}
	}
	cin>>k;
	bfs();
	for(int i=0;i<n;++i){
		for(int j=0;j<m;++j)cout<<g[i][j];
		cout<<"\n";
	}
	return 0;
}

