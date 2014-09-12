#ifndef EXCEPTIONHANDLER_HPP
#define EXCEPTIONHANDLER_HPP

#include "triangle/core/MinimalAreaTriangle.hpp"

#include <iostream>
#include <string>


namespace triangle {

    //! Exception handler class
    class ExceptionHandler {

        public:

            //! Print the error message
            /*! The error message is printed using the ex.what() method
             *
             * \param ex Exception
             */
            static void printErrorMessage(const std::exception &ex) {
                std::cerr << std::endl << ERR_MSG << std::endl
                          << std::endl << ex.what()
                          << std::endl << std::endl;
            }

    };

};


#endif
