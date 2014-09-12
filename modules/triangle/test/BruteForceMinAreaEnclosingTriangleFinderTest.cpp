#include "MinAreaEnclosingTriangleFinderTest.hpp"

#include <limits>
#include <vector>

using namespace triangle;
using namespace triangletest;


namespace triangletest {

    //! Class for testing the brute force minimum enclosing triangle algorithm
    class BruteForceMinAreaEnclosingTriangleFinderTest : public MinAreaEnclosingTriangleFinderTest {

        protected:

           //! Run the test for the given set of points
           void RunTest() override;

    };

    void BruteForceMinAreaEnclosingTriangleFinderTest::RunTest() {
        area = LinearMinAreaEnclosingTriangleFinder().find(points, triangle);

        cv::convexHull(points, convexHull);
    }

};


// Tests
TEST_F(BruteForceMinAreaEnclosingTriangleFinderTest, TestNoPoints) {
    EXPECT_THROW(TestNoPoints(), std::exception);
}

TEST_F(BruteForceMinAreaEnclosingTriangleFinderTest, TestNegativeCoordinates) {
    EXPECT_TRUE(TestPointsWithNegativeXCoordinate());
    EXPECT_TRUE(TestPointsWithNegativeYCoordinate());
    EXPECT_TRUE(TestPointsWithNegativeCoordinates());
}

TEST_F(BruteForceMinAreaEnclosingTriangleFinderTest, TestVaryingNumberOfPoints) {
    EXPECT_TRUE(TestOnePoint());
    EXPECT_TRUE(TestTwoPoints());
    EXPECT_TRUE(TestThreePoints());
    EXPECT_TRUE(TestMorePoints());
    EXPECT_TRUE(TestMorePointsAndNonEmptyTriangle());
}

TEST_F(BruteForceMinAreaEnclosingTriangleFinderTest, TestRandomPoints) {
    EXPECT_TRUE(TestRandomPoints());
}


// Main method
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
