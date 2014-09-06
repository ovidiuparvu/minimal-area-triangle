#ifndef IOEXCEPTION_HPP
#define IOEXCEPTION_HPP

#include "triangle/exception/MinimalAreaTriangleException.hpp"

#include <string>


namespace triangle {

    //! Class for representing input and output exceptions
    class IOException : public MinimalAreaTriangleException {

        public:

            IOException() {}

            explicit IOException(const std::string &file, int line, const std::string &msg) {
                constructExplanatoryString<const std::string &>(file, line, msg);
            }

            explicit IOException(const std::string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
