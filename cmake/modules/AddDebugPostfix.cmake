# Add a DEBUG_POSTFIX to the given target

function(AddDebugPostfix targetName)
    set_target_properties(${targetName}
        PROPERTIES 
        DEBUG_POSTFIX _d
    )        
endfunction(AddDebugPostfix)

