#ifndef TESTEXCEPTION_HPP
#define TESTEXCEPTION_HPP

#include "triangle/exception/MinimalAreaTriangleException.hpp" 

#include <string>

using namespace std;


namespace triangle {

    //! Class for representing testing exceptions
    class TestException : public MinimalAreaTriangleException {

        public:

            TestException() {}

            explicit TestException(const string &file, int line, const string &msg) {
                constructExplanatoryString<const string &>(file, line, msg);
            }

            explicit TestException(const string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
