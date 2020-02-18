#include<bits/stdc++.h>
using namespace std;

struct info
{
    int est_time;
    int ac_time;
    info( int _est_time , int _ac_time)
    {
        est_time = _est_time;
        ac_time = _ac_time;
    }

};

struct myComparator
{
    bool operator() (info* lhs, info * rhs)
    {
        cout<<lhs<<" "<<rhs<<endl;
        cout<<lhs->est_time<<" in comparator "<<rhs->est_time<<endl;
        if(lhs->est_time < rhs->est_time)
            return true;
        else if(rhs->est_time > lhs->est_time)
            return false;
        else
        {
            return lhs->ac_time < rhs->ac_time;
        }
    }
};

int main()
{
    priority_queue< info*, vector< info*>, myComparator > pq;
    info one = info(100, 90);
    pq.push(&one);
    info one1 = info(100, 100);
    pq.push(&one1);
    info one2 = info(110, 80);
    pq.push(&one2);
    info one3 = info(100, 110);
    pq.push(&one3);
    cout<<pq.size()<<endl;
    while(!pq.empty()){
    cout<<pq.top()->est_time<<" "<<pq.top()->ac_time<<endl;
    pq.pop();
    }
}

