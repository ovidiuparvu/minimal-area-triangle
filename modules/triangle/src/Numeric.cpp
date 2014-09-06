#include "triangle/Numeric.hpp"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <limits>
#include <vector>

using namespace triangle;


double Numeric::epsilon = 1E-5;


bool Numeric::greaterOrEqual(double number1, double number2) {
    return ((number1 > number2) || (almostEqual(number1, number2)));
}

bool Numeric::lessOrEqual(double number1, double number2) {
    return ((number1 < number2) || (almostEqual(number1, number2)));
}

bool Numeric::almostEqual(double number1, double number2) {
    return (abs(number1 - number2) <= (epsilon * std::max(1.0, std::max(abs(number1), abs(number2)))));
}

int Numeric::sign(double number) {
    return (number > 0) ? 1 : ((number < 0) ? -1 : 0);
}
