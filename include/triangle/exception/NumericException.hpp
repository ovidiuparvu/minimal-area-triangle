#ifndef NUMERICEXCEPTION_HPP
#define NUMERICEXCEPTION_HPP

#include "triangle/exception/MinimalAreaTriangleException.hpp"

#include <string>

using namespace std;


namespace triangle {

    //! Class for representing algorithm exceptions
    class NumericException : public MinimalAreaTriangleException {

        public:

            NumericException() {}

            explicit NumericException(const string &file, int line, const string &msg) {
                constructExplanatoryString<const string &>(file, line, msg);
            }

            explicit NumericException(const string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
