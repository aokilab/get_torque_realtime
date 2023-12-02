#include <stdio.h>
#include <math.h>
#include <iostream>

struct Result {
    double re1;
    double re2;
    double re3;
};

Result a(double value) {
    Result angle;
    angle.re1 = pow(value, 1);
    angle.re2 = pow(value, 2);
    angle.re3 = pow(value, 3);

    return angle;
}

int main() {

    Result a_result;

     a_result = a(2.0);

    std::cout << "a1: " << a_result.re1 << std::endl;
    std::cout << "a1: " << a_result.re2 << std::endl;
    std::cout << "a1: " << a_result.re3 << std::endl;

    return 0;
}
