#ifndef NUMERIC_HPP
#define NUMERIC_HPP

#include "triangle/exception/NumericException.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;


namespace triangle {

    //! Class for processing numeric (shorts, ints, floats, doubles etc.) expressions
    class Numeric {

        private:

            static double epsilon;  /*!< Value of epsilon used to compare two real numbers */

        public:

            //! Check if the first number is greater than or equal to the second number
            /*!
             * \param number1 The first number
             * \param number2 The second number
             */
            static bool greaterOrEqual(double number1, double number2);

            //! Check if the first number is less than or equal to the second number
            /*!
             * \param number1 The first number
             * \param number2 The second number
             */
            static bool lessOrEqual(double number1, double number2);

            //! Check if the two numbers are equal (almost)
            /*!
             * The expression for determining if two real numbers are equal is:
             * if (Abs(x - y) <= EPSILON * Max(1.0f, Abs(x), Abs(y))).
             *
             * \param number1 First number
             * \param number2 Second number
             */
            static bool almostEqual(double number1, double number2);

            //! Return the sign of the number
            /*!
             * The sign function returns:
             *  -1, if number < 0
             *  +1, if number > 0
             *  0, otherwise
             *
             *  \param number The considered number
             */
            static int sign(double number);

        private:

            //! Check if the given number is positive
            /*!
             * \param number The given number
             */
            template <typename T>
            static bool isPositive(T number) {
                return (number > 0);
            }

    };

};


#endif
