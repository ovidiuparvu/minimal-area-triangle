#ifndef MINIMALAREATRIANGLE_HPP
#define MINIMALAREATRIANGLE_HPP

#include <string>

// Core definitions available for the entire minimal area triangle project


// Platform dependent macro definitions

#if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
    #define MINIMAL_AREA_TRIANGLE_WINDOWS
#elif defined (__unix__)
    #define MINIMAL_AREA_TRIANGLE_UNIX
#endif


namespace triangle {

    // Execution return codes
    const int EXEC_SUCCESS_CODE = 0;
    const int EXEC_ERR_CODE     = 1;

    // Error related definitions
    const std::string ERR_MSG = "An error occurred: ";

    // Specific error messages
    const std::string ERR_UNDEFINED_ENUM_VALUE = "The provided enumeration value is invalid. Please use one of the available enumeration values instead.";

    const std::string ERR_INDEX_OUT_OF_BOUNDS_BEGIN = "The provided index value (";
    const std::string ERR_INDEX_OUT_OF_BOUNDS_END   = ") is invalid. Please change.";

};


#endif
