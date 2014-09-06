#------------------------------------------------------------
# Include directories
#------------------------------------------------------------

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


#------------------------------------------------------------
# Add library
#------------------------------------------------------------

# Util library

set(LIB_SRC
    src/Geometry2D.cpp
    src/LinearMinAreaEnclosingTriangleFinder.cpp
    src/Numeric.cpp
)

add_library(util STATIC ${LIB_SRC})

target_link_libraries(util ${OpenCV_LIBRARIES})
target_link_libraries(util ${Boost_LIBRARIES})


# MinAreaEnclosingTriangleFinderTest

set(MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_SRC
    test/MinAreaEnclosingTriangleFinderTest.cpp
)

set(MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_LINK_LIBS
    util
    ${OpenCV_LIBRARIES}
    gtest
    gtest_main
)

AddUnitTest(MinAreaEnclosingTriangleFinderTest "${MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_SRC}" "${MIN_ENCLOSING_TRIANGLE_FINDER_TEST_LINK_LIBS}")


# MinAreaEnclosingTriangleFinderSample

set(MIN_AREA_ENCLOSING_TRIANGLE_FINDER_SAMPLE_SRC
    sample/MinAreaEnclosingTriangleFinderSample.cpp
)

add_executable(MinAreaEnclosingTriangleFinderSample ${MIN_AREA_ENCLOSING_TRIANGLE_FINDER_SAMPLE_SRC})
AddSample(MinAreaEnclosingTriangleFinderSample)

target_link_libraries(MinAreaEnclosingTriangleFinderSample util)
target_link_libraries(MinAreaEnclosingTriangleFinderSample ${OpenCV_LIBRARIES})