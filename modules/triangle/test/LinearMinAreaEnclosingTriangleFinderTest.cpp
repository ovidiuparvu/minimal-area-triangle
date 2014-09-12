#include "MinAreaEnclosingTriangleFinderTest.hpp"

#include <limits>
#include <vector>

using namespace triangle;
using namespace triangletest;


namespace triangletest {

    //! Class for testing the linear minimum enclosing triangle algorithm
    class LinearMinAreaEnclosingTriangleFinderTest : public MinAreaEnclosingTriangleFinderTest {

        protected:

           //! Run the test for the given set of points
           void RunTest() override;

    };

    void LinearMinAreaEnclosingTriangleFinderTest::RunTest() {
        area = LinearMinAreaEnclosingTriangleFinder().find(points, triangle);

        cv::convexHull(points, convexHull);
    }

};


// Tests
TEST_F(LinearMinAreaEnclosingTriangleFinderTest, TestNoPoints) {
    EXPECT_THROW(TestNoPoints(), std::exception);
}

TEST_F(LinearMinAreaEnclosingTriangleFinderTest, TestNegativeCoordinates) {
    EXPECT_TRUE(TestPointsWithNegativeXCoordinate());
    EXPECT_TRUE(TestPointsWithNegativeYCoordinate());
    EXPECT_TRUE(TestPointsWithNegativeCoordinates());
}

TEST_F(LinearMinAreaEnclosingTriangleFinderTest, TestVaryingNumberOfPoints) {
    EXPECT_TRUE(TestOnePoint());
    EXPECT_TRUE(TestTwoPoints());
    EXPECT_TRUE(TestThreePoints());
    EXPECT_TRUE(TestMorePoints());
    EXPECT_TRUE(TestMorePointsAndNonEmptyTriangle());
}

TEST_F(LinearMinAreaEnclosingTriangleFinderTest, TestRandomPoints) {
    EXPECT_TRUE(TestRandomPoints());
}


// Main method
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
