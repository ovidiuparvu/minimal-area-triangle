#ifndef MINIMALAREATRIANGLEEXCEPTION_HPP
#define MINIMALAREATRIANGLEEXCEPTION_HPP

#include "triangle/core/MinimalAreaTriangle.hpp"

#include <stdexcept>
#include <string>
#include <sstream>
#include <typeinfo>

#define MAT_throw(ex, msg)                               (throw ex(__FILE__, __LINE__, msg))
#define MAT_throw_detailed(ex, startMsg, msg, endMsg)    (throw ex(__FILE__, __LINE__, startMsg + msg + endMsg))


namespace triangle {

    //! Parent exception class for the project
    class MinimalAreaTriangleException : public std::runtime_error {

        protected:

            std::string message;             /*!< The raw message of the exception */
            std::string explanatoryString;   /*!< User friendly exception message */

        public:

            MinimalAreaTriangleException() : std::runtime_error("") {}

            explicit MinimalAreaTriangleException(const std::string &file, int line,
                                                  const std::string &msg)
                : std::runtime_error(msg) {}
            explicit MinimalAreaTriangleException(const std::string &file, int line,
                                                  const char *msg)
                : std::runtime_error(msg) {}

            //! Returns an explanatory std::string
            const char* what() const noexcept override {
                return explanatoryString.c_str();
            }

            //! Return the raw message of the exception
            std::string rawMessage() const noexcept {
                return message;
            }

        protected:

            //! Construct the explanatory std::string
            /*!
             * \param file  File where the error occurred
             * \param line  Line number where the error occurred
             * \param msg   Error message
             */
            template <typename T>
            void constructExplanatoryString(const std::string &file, int line, T msg) {
                std::stringstream strStream;

                strStream << "File: " << file << ", " << std::endl
                          << "Line: " << line << ", " << std::endl
                          << "Type: " << typeid(*this).name() << ", " << std::endl
                          << "Error message: " << msg;

                explanatoryString = strStream.str();
                message           = std::string(msg);
            }

    };

};


#endif
