#ifndef INVALIDINPUTEXCEPTION_HPP
#define INVALIDINPUTEXCEPTION_HPP

#include "triangle/exception/IOException.hpp"

#include <string>


namespace triangle {

    //! Class for representing invalid input exceptions
    class InvalidInputException : public IOException {

        public:

            InvalidInputException() {}

            explicit InvalidInputException(const std::string &file, int line, const std::string &msg) {
                constructExplanatoryString<const std::string &>(file, line, msg);
            }

            explicit InvalidInputException(const std::string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
