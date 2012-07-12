#include "dynamictimewarping.h"

using namespace std;
using namespace cv;

DynamicTimeWarping::DynamicTimeWarping()
{
}


void DynamicTimeWarping::match(double &cost, vector<IndexPair> &index, const vector<double> &contour1, const vector<double> &contour2, int metric)
{
    if (metric == EUCLEDIAN){
        matchEuc(cost,index,contour1,contour2);
    }
}

void DynamicTimeWarping::matchEuc(double &cost, vector<IndexPair> &index, const vector<double> &contour1, const vector<double> &contour2)
{
    //allocate distance matrix
    int M = contour1.size(), N = contour2.size();
    double pairD[M*N];
    double D[M*N];
    int idx;
    vector<double>::const_iterator i1,i2;

    i1 = contour1.begin();

    //compute distancies
    for(int i=0; i<M; i++, i1++){
        i2 = contour2.begin();
        for(int j=0; j<N; j++, i2++){
            pairD[i*N+j] = distance(*i1,*i2);
        }
    }

    //init array boundaries
    D[0] = pairD[0];
    for(int i=1; i<M; i++){
        D[i*N] = D[(i-1)*N] + pairD[i*N];
    }
    for(int j=1; j<N; j++){
        D[j] = D[j-1] + pairD[j];
    }
    //compute everything else
    for(int i=2; i<M; i++){
        for(int j=2; j<N; j++){
            D[i*N+j] = pairD[i*N+j]+min(idx,D[(i-1)*N+j],D[i*N+(j-1)],D[(i-1)*N+(j-1)]);
        }
    }

    int i=M,j=N;

    //compute shortest path
    while(i+j>0){
        if((i-1)==0){ j = j-1; }
        else{ if ((j-1)==0){ i = i-1;}
            else{ min(idx,D[(i-1)*N+j],D[i*N+(j-1)],D[(i-1)*N+(j-1)]);
                 if(idx == 0){i = i-1;}
                 else{if(idx==1){j = j-1;}
                    else{i=i-1; j=j-1;}}
                }
            }
        index.push_back(IndexPair(i,j));

    }

    //set cost
    cost = D[M*N-1];
}
