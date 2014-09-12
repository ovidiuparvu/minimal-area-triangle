#ifndef STRINGMANIPULATOR_HPP
#define STRINGMANIPULATOR_HPP

#include <string>
#include <sstream>


namespace triangle {

    namespace util {

        //! Class for manipulating strings
        class StringManipulator {

            public:

                //! Convert the variable to a string
                /*!
                 * \param variable Variable
                 */
                template <typename T>
                static std::string toString(T variable) {
                    std::ostringstream stringStream;

                    stringStream << variable;

                    return stringStream.str();
                }

        };

    };

};


#endif
