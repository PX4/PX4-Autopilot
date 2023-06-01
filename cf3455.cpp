//I am 1 oF MY KiND .PK

#include<bits/stdc++.h>
using namespace std;
#define int long long
#define vec vector<long long>
#define Map map<long long,long long>
#define Unmap unordered_map<long long,long long>
#define Set set<long long>
#define Unset unordered_set<long long>
#define push push_back
#define din>>n int n;cin>>n


void Solve()
{

    int n;
    cin>>n;
    int a[n];
    int b[n];
    for(int i=0; i<=n-1; i++){
        cin>>a[i];
    }
    for(int i=0; i<=n-1; i++){
        cin>>b[i];
    }

    int st=0,ed=1;
    int cnt=0;
    for (int i=0; i<n; i++){
        for (int j=i+1; j<n; j++){
            if (a[i]*a[j]==b[i]+b[j]){
                cnt++;
            }
        }
    }
    cout<<cnt<<endl;

}

signed main(){

    int t;
    cin>>t;
    while (t--){

        Solve();

    }

return 0;
}