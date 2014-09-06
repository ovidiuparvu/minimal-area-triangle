# Add an executable install target

function(AddExecutableInstallTarget executableName executableSources)
    add_executable(${executableName} ${executableSources})
    
    install(TARGETS ${executableName} DESTINATION bin)      
endfunction(AddExecutableInstallTarget)

