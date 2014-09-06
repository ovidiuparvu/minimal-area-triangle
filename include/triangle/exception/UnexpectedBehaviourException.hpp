#ifndef UNEXPECTEDBEHAVIOUREXCEPTION_HPP
#define UNEXPECTEDBEHAVIOUREXCEPTION_HPP

#include "triangle/exception/AlgorithmException.hpp"

#include <string>

using namespace std;


namespace triangle {

    //! Class for representing unexpected behaviour exceptions
    class UnexpectedBehaviourException : public AlgorithmException {

        public:

            UnexpectedBehaviourException() {}

            explicit UnexpectedBehaviourException(const string &file, int line, const string &msg) {
                constructExplanatoryString<const string &>(file, line, msg);
            }

            explicit UnexpectedBehaviourException(const string &file, int line, const char *msg) {
                constructExplanatoryString<const char *>(file, line, msg);
            }

    };

};


#endif
