#include <iostream>
#include "quest.hpp"
using namespace std;
int main()
{
    Matrix B1 = {{372.221}, {782.933}, {11.282}};
    Matrix B2 = {{30.722}, {732.773}, {12.43}};
    Matrix R1 = {{21.772}, {-233.71}, {92.902}};
    Matrix R2 = {{73.662}, {93.102}, {44.372}};

    float_t w1 = 0.5f;
    float_t w2 = 0.5f;

    Matrix quarternion = quest(B1, B2, R1, R2, w1, w2);
    cout << quarternion << endl;
    return 0;
}