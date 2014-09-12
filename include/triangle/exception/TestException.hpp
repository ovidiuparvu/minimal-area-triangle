#ifndef TESTEXCEPTION_HPP
#define TESTEXCEPTION_HPP

#include "triangle/exception/MinimalAreaTriangleException.hpp" 

#include <string>


namespace triangle {

    //! Class for representing testing exceptions
    class TestException : public MinimalAreaTriangleException {

        public:

            TestException() {}

            explicit TestException(const std::string &file, int line, const std::string &msg) {
                constructExplanatoryString<const std::string &>(file, line, msg);
            }

            explicit TestException(const std::string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
