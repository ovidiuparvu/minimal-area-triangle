#include "triangle/core/MinimalAreaTriangleTest.hpp"
#include "triangle/util/Geometry2D.hpp"

#include <limits>
#include <vector>

using namespace std;
using namespace cv;
using namespace triangle;
using namespace triangletest;

const double DOUBLE_COMP_ERROR = 1E-6;


// Tests
TEST(Geometry2D, TriangleArea) {
    EXPECT_NEAR(0, Geometry2D::areaOfTriangle(Point2f(1, 2), Point2f(2, 3), Point2f(3, 4)), DOUBLE_COMP_ERROR);
    EXPECT_NEAR(9.5, Geometry2D::areaOfTriangle(Point2f(5, 2), Point2f(2, 3), Point2f(3, 9)), DOUBLE_COMP_ERROR);
    EXPECT_NEAR(3.7, Geometry2D::areaOfTriangle(Point2f(5, 5), Point2f(2.4, 0), Point2f(3, 4)), DOUBLE_COMP_ERROR);
    EXPECT_NEAR(24.5, Geometry2D::areaOfTriangle(Point2f(0, 4.9), Point2f(0, 0), Point2f(10, 10)), DOUBLE_COMP_ERROR);
}

TEST(Geometry2D, PointOnLineSegment) {
    EXPECT_TRUE(Geometry2D::isPointOnLineSegment(Point2f(2, 2), Point2f(2, 2), Point2f(10, 4)));
    EXPECT_TRUE(Geometry2D::isPointOnLineSegment(Point2f(10, 4), Point2f(2, 2), Point2f(10, 4)));
    EXPECT_TRUE(Geometry2D::isPointOnLineSegment(Point2f(6, 3), Point2f(2, 2), Point2f(10, 4)));
    EXPECT_FALSE(Geometry2D::isPointOnLineSegment(Point2f(4, 3), Point2f(2, 2), Point2f(10, 4)));
    EXPECT_FALSE(Geometry2D::isPointOnLineSegment(Point2f(6, 4), Point2f(2, 2), Point2f(10, 4)));
}


// Main method
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
