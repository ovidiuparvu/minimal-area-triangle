# Get the current date

function(GetCurrentDate RESULT)
    IF (WIN32)
        EXECUTE_PROCESS(COMMAND "cmd" " /C date /T" OUTPUT_VARIABLE DATE)
        string(REGEX REPLACE "(..)/(..)/(....).*" "\\1.\\2.\\3" FORMATTED_DATE ${DATE})
    ELSEIF(UNIX)
        EXECUTE_PROCESS(COMMAND "date" "+%d/%m/%Y" OUTPUT_VARIABLE DATE)
        string(REGEX REPLACE "(..)/(..)/(....).*" "\\1.\\2.\\3" FORMATTED_DATE ${DATE})
    ELSE (WIN32)
        MESSAGE(SEND_ERROR "date not implemented")
        SET(${RESULT} 000000)
    ENDIF (WIN32)
    
    SET(${RESULT} ${FORMATTED_DATE} PARENT_SCOPE)
endfunction(GetCurrentDate)