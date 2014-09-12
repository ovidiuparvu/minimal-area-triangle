#------------------------------------------------------------
# Include directories
#------------------------------------------------------------

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


#------------------------------------------------------------
# Add library
#------------------------------------------------------------

# Util library

set(LIB_SRC
    src/Filesystem.cpp
    src/Geometry2D.cpp
    src/Numeric.cpp
)

add_library(util STATIC ${LIB_SRC})

target_link_libraries(util ${OpenCV_LIBRARIES})
target_link_libraries(util ${Boost_LIBRARIES})