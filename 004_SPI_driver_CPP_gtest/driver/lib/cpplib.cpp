#include "cpplib.h"
#include <climits>
using namespace std;


string cpp_lib::print_hello_world(){
    return "Hello World";
}

int cpp_lib::find_max(const vector<int> &input) {
    if(input.empty()) {
        return -1;
    }
    int maxVal = INT_MIN;
    for(auto &data : input) {
        if(data > maxVal) {
            maxVal = data;
        }
    }
    return maxVal;
}