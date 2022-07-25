#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

//Dijkstra's algo

class node{
    public:
        int name, value=10000, f=10000, heur;
        vector<int> length;
        vector<node*> suc;
};

bool isin(vector<node*> open, node* suc){
    for(unsigned int i=0; i<open.size(); i++){
        if(open[i]==suc){
            return true;
        }
    }
    return false;
}

void dijkstra(node **m, int n, int start1, int start2, int end1, int end2, vector<node*> &shortest){
    vector<node*> openlist, closelist;
    for(int i=0; i<n; i++){
        for(int j=0; j<n; j++){
            if(m[i][j].name!=-1){
                openlist.push_back(&m[i][j]);
            }
        }
    }
    for(int j=0; j<m[start1][start2].suc.size(); j++){
        if(m[start1][start2].suc[j]->name!=-1){
            m[start1][start2].suc[j]->f=1+m[start1][start2].suc[j]->heur;
            m[start1][start2].suc[j]->value=1;
        }
    }
    while(openlist.size()>0){
        int mini=0, min1=openlist[0]->f;
        for(unsigned int o=0; o<openlist.size(); o++){
            if(openlist[o]->f<min1){
                min1=openlist[o]->f;
                mini=o;
            }
        }
        cout<<openlist[mini]->name<<" ";
        if(openlist[mini]->name!=-1){
            closelist.push_back(openlist[mini]);
            for(int l=0; l<openlist[mini]->suc.size(); l++){
                if(isin(openlist, openlist[mini]->suc[l])&&openlist[mini]->suc[l]->name!=-1){
                    if(openlist[mini]->suc[l]==&m[end1][end2]){
                        node* pos=openlist[mini]->suc[l];
                        shortest.push_back(pos);
                        cout<<endl;
                        while(pos!=&m[start1][start2]){
                            int min2=pos->suc[0]->value;
                            node* minj=pos->suc[0];
                            for(unsigned int a=0; a<pos->suc.size(); a++){
                                if(pos->suc[a]->value<min2&&pos->suc[a]->name!=-1){
                                    min2=pos->suc[a]->value;
                                    minj=pos->suc[a];
                                }
                            }
                            shortest.push_back(minj);
                            //cout<<minj->name<<" ";
                            pos=minj;
                        }
                        return;
                    }
                }
                if(isin(openlist, openlist[mini]->suc[l])&&openlist[mini]->suc[l]->name!=-1){
                    if(openlist[mini]->suc[l]->f>openlist[mini]->value+openlist[mini]->length[l]+openlist[mini]->suc[l]->heur){
                        openlist[mini]->suc[l]->value=openlist[mini]->value+openlist[mini]->length[l];
                        openlist[mini]->suc[l]->f=openlist[mini]->value+openlist[mini]->length[l]+openlist[mini]->suc[l]->heur;
                    }
                }
                if(isin(closelist, openlist[mini]->suc[l])&&openlist[mini]->suc[l]->name!=-1){
                    if(openlist[mini]->suc[l]->f>openlist[mini]->value+openlist[mini]->length[l]+openlist[mini]->suc[l]->heur){
                        openlist.push_back(openlist[mini]->suc[l]);
                    }
                }
            }
            openlist.erase(openlist.begin()+mini);
        }
    }
}

int main(){
    int n;
    cout<<"Enter the size of the grid";
    cin>>n;
    node nodelist[n][n];
    cout<<"Enter the names of the nodes and '-1' for an obstacle"<<endl;
    for(int i=0; i<n; i++){
        for(int j=0; j<n; j++){
            cin>>nodelist[i][j].name;
        }
    }
    for(int i=0; i<n; i++){
        for(int j=0; j<n; j++){
            if(i!=0){
                nodelist[i][j].suc.push_back(&nodelist[i-1][j]);
                nodelist[i][j].length.push_back(1);
            }
            if(i!=n-1){
                nodelist[i][j].suc.push_back(&nodelist[i+1][j]);
                nodelist[i][j].length.push_back(1);
            }
            if(j!=0){
                nodelist[i][j].suc.push_back(&nodelist[i][j-1]);
                nodelist[i][j].length.push_back(1);
            }
            if(j!=n-1){
                nodelist[i][j].suc.push_back(&nodelist[i][j+1]);
                nodelist[i][j].length.push_back(1);
            }
        }
    }

    cout<<"Enter the starting node coordinates"<<endl;
    int x, y;
    cin>>x>>y;
    nodelist[x][y].value=0;
    nodelist[x][y].f=nodelist[x][y].heur;
    cout<<"Ending node coordinates"<<endl;
    int alpha, beta;
    cin>>alpha>>beta;
    for(int p=0; p<n; p++){
        for(int q=0; q<n; q++){
            if(nodelist[p][q].name!=-1){
                nodelist[p][q].heur=(abs(p-alpha)+abs(q-beta));
            }
        }
    }
    node *nodelistcopy[n];
    for(int l=0; l<n; l++){
        nodelistcopy[l]=nodelist[l];
    }
    vector<node*> answer;
    dijkstra(nodelistcopy, n, x, y, alpha, beta, answer);
    //cout<<answer.size()<<endl;
    for(int s=answer.size()-1; s>=0; s--){
        cout<<answer[s]->name<<" ";
    }
}
