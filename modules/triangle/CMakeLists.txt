#------------------------------------------------------------
# Include directories
#------------------------------------------------------------

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


#------------------------------------------------------------
# Add executables
#------------------------------------------------------------

# BruteForceMinAreaEnclosingTriangleFinderTest

set(BRUTE_FORCE_MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_SRC
    src/BruteForceMinAreaEnclosingTriangleFinder.cpp
    src/LinearMinAreaEnclosingTriangleFinder.cpp
    src/MinAreaEnclosingTriangleFinder.cpp

    test/BruteForceMinAreaEnclosingTriangleFinderTest.cpp
)

set(BRUTE_FORCE_MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_LINK_LIBS
    util
    gtest
    gtest_main
)

AddUnitTest(BruteForceMinAreaEnclosingTriangleFinderTest "${BRUTE_FORCE_MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_SRC}" "${BRUTE_FORCE_MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_LINK_LIBS}")


# LinearMinAreaEnclosingTriangleFinderTest

set(LINEAR_MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_SRC
    src/BruteForceMinAreaEnclosingTriangleFinder.cpp
    src/LinearMinAreaEnclosingTriangleFinder.cpp
    src/MinAreaEnclosingTriangleFinder.cpp
    
    test/LinearMinAreaEnclosingTriangleFinderTest.cpp
)

set(LINEAR_MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_LINK_LIBS
    util
    gtest
    gtest_main
)

AddUnitTest(LinearMinAreaEnclosingTriangleFinderTest "${LINEAR_MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_SRC}" "${LINEAR_MIN_AREA_ENCLOSING_TRIANGLE_FINDER_TEST_LINK_LIBS}")


# RandomPolygonsCorrectnessTest

set(RANDOM_POLYGONS_CORRECTNESS_TEST_SRC
    src/BruteForceMinAreaEnclosingTriangleFinder.cpp
    src/LinearMinAreaEnclosingTriangleFinder.cpp
    src/MinAreaEnclosingTriangleFinder.cpp
    
    test/RandomPolygonsCorrectnessTest.cpp
)

add_executable(RandomPolygonsCorrectnessTest ${RANDOM_POLYGONS_CORRECTNESS_TEST_SRC})

target_link_libraries(RandomPolygonsCorrectnessTest util)


# MinEnclosingTriangleFinderSample

set(MIN_AREA_ENCLOSING_TRIANGLE_FINDER_SAMPLE_SRC
    src/BruteForceMinAreaEnclosingTriangleFinder.cpp
    src/LinearMinAreaEnclosingTriangleFinder.cpp
    src/MinAreaEnclosingTriangleFinder.cpp
    
    sample/MinAreaEnclosingTriangleFinderSample.cpp
)

add_executable(MinAreaEnclosingTriangleFinderSample ${MIN_AREA_ENCLOSING_TRIANGLE_FINDER_SAMPLE_SRC})
AddSample(MinAreaEnclosingTriangleFinderSample)

target_link_libraries(MinAreaEnclosingTriangleFinderSample util)