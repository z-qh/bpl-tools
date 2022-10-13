#include <iostream> 
#include <vector>
#include <algorithm>
using namespace std;

uint32_t findMissing(vector<vector<uint32_t>> numbers) {
    for(int i = 0;i < numbers.size();i++)
    {
        if((i&1) != numbers[i][0])
            return i;
    }
    return numbers.size();
}
vector<vector<uint32_t>> generate_data(vector<uint32_t>&d){
    vector<vector<uint32_t>> data;
    for(auto p:d){
        vector<uint32_t> tmp(32, 0);
        for(int i = 0; i < 32; ++i){
            tmp[i] = (p >> i) & 1;
        }
        data.emplace_back(move(tmp));
    }
    return data;
}

int main(){
    cout << " test " << endl;
    vector<uint32_t> full{0,1,2,3,4,5,6,7};
    vector<uint32_t> lack{0,1,2,3,4,5,7};
    auto data = generate_data(lack);
    for(auto&p:data){
        for(auto&a:p)cout << a << " ";
        cout << endl;
    }
    cout << endl;
    cout << findMissing(data) << endl;
    return 0;
}