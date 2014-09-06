# Update the project build number and replace all project build numbers in the input file

function(UpdateProjectBuildNumber buildType inputFile outputFile)
    SetProjectBuildNumber(
        ${outputFile}
    )

    if(CMAKE_BUILD_TYPE STREQUAL ${buildType})
        IncrementProjectBuildNumber()
    endif(CMAKE_BUILD_TYPE STREQUAL ${buildType})         
    
    configure_file(
        ${inputFile} ${outputFile}
    )
    
    set(PROJECT_VERSION_MAJOR ${PROJECT_VERSION_MAJOR} PARENT_SCOPE)
    set(PROJECT_VERSION_MINOR ${PROJECT_VERSION_MINOR} PARENT_SCOPE)
    set(PROJECT_VERSION_BUILD ${PROJECT_VERSION_BUILD} PARENT_SCOPE)
endfunction(UpdateProjectBuildNumber)

# Set the project build number

function(SetProjectBuildNumber filepath)
    file(
        STRINGS 
        ${filepath}
        PROJECT_NUMBER_STRING
        REGEX "^PROJECT_NUMBER = [0-9]+[.][0-9]+[.][0-9]+"
    )
       
    string(REGEX MATCH " [0-9]+[.]" MAJOR ${PROJECT_NUMBER_STRING})
    string(REGEX REPLACE "[ .]+" "" PROJECT_VERSION_MAJOR ${MAJOR})
    set(PROJECT_VERSION_MAJOR ${PROJECT_VERSION_MAJOR} PARENT_SCOPE)
        
    string(REGEX MATCH "[.][0-9]+[.]" MINOR ${PROJECT_NUMBER_STRING})
    string(REGEX REPLACE "[.]+" "" PROJECT_VERSION_MINOR ${MINOR})
    set(PROJECT_VERSION_MINOR ${PROJECT_VERSION_MINOR} PARENT_SCOPE)
    
    string(REGEX MATCH "[.][0-9]+$" BUILD ${PROJECT_NUMBER_STRING})
    string(REGEX REPLACE "[.]+" "" PROJECT_VERSION_BUILD ${BUILD})
    set(PROJECT_VERSION_BUILD ${PROJECT_VERSION_BUILD} PARENT_SCOPE)
endfunction(SetProjectBuildNumber)

# Increment the project build number

function(IncrementProjectBuildNumber)
    math(EXPR PROJECT_VERSION_BUILD "${PROJECT_VERSION_BUILD} + 1") 
    set(PROJECT_VERSION_BUILD ${PROJECT_VERSION_BUILD} PARENT_SCOPE)  
endfunction(IncrementProjectBuildNumber)