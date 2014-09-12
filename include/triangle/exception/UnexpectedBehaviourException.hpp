#ifndef UNEXPECTEDBEHAVIOUREXCEPTION_HPP
#define UNEXPECTEDBEHAVIOUREXCEPTION_HPP

#include "triangle/exception/AlgorithmException.hpp"

#include <string>


namespace triangle {

    //! Class for representing unexpected behaviour exceptions
    class UnexpectedBehaviourException : public AlgorithmException {

        public:

            UnexpectedBehaviourException() {}

            explicit UnexpectedBehaviourException(const std::string &file, int line, const std::string &msg) {
                constructExplanatoryString<const std::string &>(file, line, msg);
            }

            explicit UnexpectedBehaviourException(const std::string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
