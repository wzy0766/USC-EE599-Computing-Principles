#include <iostream>
#include "practice_examples.h"
#include "limits"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <map>

using namespace std;

std::vector<char> practice_examples::Unfold(const vector<char> &A, const vector<int> &R) {
    std::vector<char> res;
    int n = A.size();
    for (int i = 0; i < n; ++i) {
        for(int j = 0; j < R[i]; ++j)
        return res.push_back(A[i]);
    }
    return res;
}

int CountUnique(const vector<int> &A) {
    set<int> s;
    for (const auto &x : A) {
        s.insert(x);
    }
    return s.size();
}

int CountRepeating(const vector<int> &A) {
    map<int, int> count;
    for (const auto &x : A) {
        ++count[x];
    }
    int ans = 0;
    for (auto x : count) {
        ans += (x.second > 1);
    }
    return ans;
}
