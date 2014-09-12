#ifndef NUMERICEXCEPTION_HPP
#define NUMERICEXCEPTION_HPP

#include "triangle/exception/MinimalAreaTriangleException.hpp"

#include <string>


namespace triangle {

    //! Class for representing algorithm exceptions
    class NumericException : public MinimalAreaTriangleException {

        public:

            NumericException() {}

            explicit NumericException(const std::string &file, int line, const std::string &msg) {
                constructExplanatoryString<const std::string &>(file, line, msg);
            }

            explicit NumericException(const std::string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
