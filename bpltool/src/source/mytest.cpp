#include <ext/pb_ds/assoc_container.hpp>
#include <ext/pb_ds/trie_policy.hpp>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
using namespace std;
using namespace __gnu_pbds;

string binaryAdd(string A, string B)
{
    string res;
    int acc = 0;
    int ai = A.size() - 1;
    int bi = B.size() - 1;
    for (; ai >= 0 || bi >= 0;)
    {
        int a, b;
        if (ai < 0)
            a = 0, b = B[bi];
        else if (bi < 0)
            b = 0, a = A[ai];
        else 
            b = B[bi], a=A[ai];
        int now = b - '0' + a - '0' + acc;
        acc = now / 2;
        now = now % 2;
        res.push_back(now + '0');
        ai--, bi--;
    }
    while (acc != 0)
    {
        res.push_back(acc % 2 + '0');
        acc /= 2;
    }
    return res;
}

int main()
{
    cout << binaryAdd("101", "100") << endl;
    return 0;
}
