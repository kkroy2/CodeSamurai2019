
#include <iostream>
#include <list>
#include <queue>

using namespace std;

typedef struct miasto
{
    int nr;
    int koszt;
} *miasto_wsk;

struct MyComparator {
    bool operator() (miasto_wsk arg1, miasto_wsk arg2) {
        return arg1->koszt < arg2->koszt; //calls your operator
    }
};


int main()
{
    //priority_queue<miasto_wsk, vector<miasto_wsk>, myComparator> q;
    priority_queue<miasto_wsk, vector<miasto_wsk>, MyComparator> q;
    miasto_wsk mi;
    mi = new miasto;
    mi->nr = 1;
    mi->koszt = 22;
    q.push(mi);
    miasto_wsk mi1;
    mi1 = new miasto;
    mi1->nr = 2;
    mi1->koszt = 50;
    q.push(mi1);
    miasto_wsk mi2;
    mi2 = new miasto;
    mi2->nr = 3;
    mi2->koszt = 1;
    q.push(mi2);

    cout << q.top()->koszt << endl;
    q.pop();
    cout << q.top()->koszt << endl;
    q.pop();
    cout << q.top()->koszt << endl;
    q.pop();
}
