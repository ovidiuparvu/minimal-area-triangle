# Update the project modified date

function(UpdateProjectModifiedDate configurationInputFile configurationOutputFile)
    GetCurrentDate(CURRENT_DATE)

    configure_file(
        ${configurationInputFile}
        ${configurationOutputFile}
    )
endfunction(UpdateProjectModifiedDate)