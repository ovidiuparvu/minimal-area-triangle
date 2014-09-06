#ifndef ALGORITHMEXCEPTION_HPP
#define ALGORITHMEXCEPTION_HPP

#include "triangle/exception/MinimalAreaTriangleException.hpp"

#include <string>

using namespace std;


namespace triangle {

    //! Class for representing algorithm exceptions
    class AlgorithmException : public MinimalAreaTriangleException {

        public:

            AlgorithmException() {}

            explicit AlgorithmException(const string &file, int line, const string &msg) {
                constructExplanatoryString<const string &>(file, line, msg);
            }

            explicit AlgorithmException(const string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
