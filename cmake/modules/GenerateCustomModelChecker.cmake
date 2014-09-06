# Generate a custom model checker according to the given configuration file

function(GenerateCustomModelChecker)
    message(AUTHOR_WARNING "Generating custom model checker corresponding to the spatial_description configuration file. Please manually update all unit tests accordingly.")

    # Check if the python interpreter is available for use
    find_package(PythonInterp REQUIRED)
    
    # Generate custom model checker considering the given configuration file
    if(PYTHONINTERP_FOUND)
        message(STATUS "Python interpreter was successfully found.")
        
        # Comma separated list of dependent python libraries
        set(PYTHON_DEPENDENT_LIBS "lxml, setuptools, jinja2")
        
        # Execute the command to check if the libraries are installed
        execute_process(
            COMMAND ${PYTHON_EXECUTABLE} -c "import ${PYTHON_DEPENDENT_LIBS};"
            RESULT_VARIABLE PYTHON_DEPENDENT_LIBS_RESULT
            OUTPUT_QUIET
            ERROR_QUIET
        )
        
        # If not return with fatal error message
        if(NOT "${PYTHON_DEPENDENT_LIBS_RESULT}" STREQUAL "0")
            message(FATAL_ERROR "Please install the required python libraries: ${PYTHON_DEPENDENT_LIBS}.")  
        endif(NOT "${PYTHON_DEPENDENT_LIBS_RESULT}" STREQUAL "0")
    endif()
endfunction(GenerateCustomModelChecker)
