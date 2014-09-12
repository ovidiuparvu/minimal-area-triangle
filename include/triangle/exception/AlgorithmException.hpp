#ifndef ALGORITHMEXCEPTION_HPP
#define ALGORITHMEXCEPTION_HPP

#include "triangle/exception/MinimalAreaTriangleException.hpp"

#include <string>


namespace triangle {

    //! Class for representing algorithm exceptions
    class AlgorithmException : public MinimalAreaTriangleException {

        public:

            AlgorithmException() {}

            explicit AlgorithmException(const std::string &file, int line, const std::string &msg) {
                constructExplanatoryString<const std::string &>(file, line, msg);
            }

            explicit AlgorithmException(const std::string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
