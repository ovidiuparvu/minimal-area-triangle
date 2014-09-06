# Add a unit test

function(AddUnitTest testName testSources testLinkLibraries)
    add_executable(${testName} ${testSources})

    set_target_properties(${testName}
        PROPERTIES 
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/test
        DEBUG_POSTFIX _d
    ) 

    add_test(NAME ${testName} COMMAND ${testName})
    
    # Uncomment instruction below if you want to execute unit tests during 
    # build process:
    # 
    # add_custom_command(
    #     TARGET ${testName} 
    #     POST_BUILD 
    #     COMMAND ctest -R "${testName}"
    # )
    
    target_link_libraries(${testName} ${testLinkLibraries})
endfunction(AddUnitTest)