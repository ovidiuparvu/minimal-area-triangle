#include "triangle/core/MinimalAreaTriangleTest.hpp"
#include "triangle/LinearMinAreaEnclosingTriangleFinder.hpp"

#include <limits>
#include <vector>

using namespace std;
using namespace cv;
using namespace triangle;
using namespace triangletest;


namespace triangletest {

    //! Class for testing the minimum enclosing triangle algorithm
    class MinEnclosingTriangleFinderTest : public MinimalAreaTriangleTest {

        protected:

            vector<Point2d> convexHull;     /*!< Convex hull of the 2D point set */

            vector<Point2d> points;         /*!< Collection of 2D points */
            vector<Point2d> triangle;       /*!< Minimum enclosing triangle */
            double area;                    /*!< Area of the minimum enclosing triangle */

        public:

           MinEnclosingTriangleFinderTest();
           ~MinEnclosingTriangleFinderTest();

           //! Test the scenario when an empty vector of points is provided
           bool TestNoPoints();

           //! Test the scenario when there exists at least one point with negative x coordinate
           bool TestPointsWithNegativeXCoordinate();

           //! Test the scenario when there exists at least one point with negative y coordinate
           bool TestPointsWithNegativeYCoordinate();

           //! Test the scenario when there exists at least one point with negative coordinates
           bool TestPointsWithNegativeCoordinates();

           //! Test the scenario when only one input point is provided
           bool TestOnePoint();

           //! Test the scenario when only two input points are provided
           bool TestTwoPoints();

           //! Test the scenario when only three input points are provided
           bool TestThreePoints();

           //! Test the scenario when more than three input points are provided
           bool TestMorePoints();

           //! Test the scenario when the output vector is not empty
           bool TestMorePointsAndNonEmptyTriangle();

           //! Test the scenario when randomly initialised vectors of input points are provided
           bool TestRandomPoints();

        protected:

           //! Run the test for the given set of points
           void RunTest() override;

           //! Check if the obtained results are valid
           void ValidateTestResults() override;

        private:

           //! Get a random number of executions
           int GetRandomNrOfExecutions();

           //! Get a random number of points
           int GetRandomNrOfPoints();

           //! Check if all the points are enclosed by the polygon
           bool ArePointsEnclosed();

           //! Check if the triangle's middle points are touching the polygon
           bool IsTriangleTouchingPolygon();

           //! Check if at least one of the triangle sides is flush with a polygon edge
           bool IsOneEdgeFlush();

        private:

           // Constants
           static const int MIN_NR_POINTS;
           static const int MAX_NR_POINTS;
           static const int MIN_NR_EXECUTIONS;
           static const int MAX_NR_EXECUTIONS;

           static const double POINT_IN_TRIANGLE_THRESH;

    };

    MinEnclosingTriangleFinderTest::MinEnclosingTriangleFinderTest() {
        area = 0;

        points.clear();
        convexHull.clear();
        triangle.clear();
    }

    MinEnclosingTriangleFinderTest::~MinEnclosingTriangleFinderTest() {
        area = 0;

        points.clear();
        convexHull.clear();
        triangle.clear();
    }

    bool MinEnclosingTriangleFinderTest::TestNoPoints() {
        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinEnclosingTriangleFinderTest::TestPointsWithNegativeXCoordinate() {
        points = vector<Point2d>({Point2d(-2, 1), Point2d(2, 2), Point2d(4, 4)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinEnclosingTriangleFinderTest::TestPointsWithNegativeYCoordinate() {
        points = vector<Point2d>({Point2d(2, -1), Point2d(2, 2), Point2d(4, 4)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinEnclosingTriangleFinderTest::TestPointsWithNegativeCoordinates() {
        points = vector<Point2d>({Point2d(-2, -1), Point2d(2, 2), Point2d(4, 4)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinEnclosingTriangleFinderTest::TestOnePoint() {
        points = vector<Point2d>({Point2d(1, 1)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinEnclosingTriangleFinderTest::TestTwoPoints() {
        points = vector<Point2d>({Point2d(1, 1), Point2d(2, 2)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinEnclosingTriangleFinderTest::TestThreePoints() {
        points = vector<Point2d>({Point2d(1, 1), Point2d(2, 2), Point2d(4, 1)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinEnclosingTriangleFinderTest::TestMorePoints() {
        points = vector<Point2d>({Point2d(1, 1), Point2d(2, 2), Point2d(4, 1), Point2d(5, 5), Point2d(6, 4)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinEnclosingTriangleFinderTest::TestMorePointsAndNonEmptyTriangle() {
        points = vector<Point2d>({Point2d(1, 1), Point2d(2, 2), Point2d(4, 1), Point2d(5, 5), Point2d(6, 4)});

        triangle.push_back(Point2d(1, 1));

        RunTest();
        ValidateTestResults();

        return ((validationFlag) && (triangle.size() == 3));
    }

    bool MinEnclosingTriangleFinderTest::TestRandomPoints() {
        srand(time(NULL));

        int nrOfRandomTrials = GetRandomNrOfExecutions();

        for (int i = 0; i < nrOfRandomTrials; i++) {
            points.clear();

            int nrOfPoints = GetRandomNrOfPoints();

            for (int j = 0; j < nrOfPoints; j++) {
                points.push_back(Point2d(rand(), rand()));
            }

            RunTest();
            ValidateTestResults();

            if (!validationFlag) {
                return false;
            }
        }

        return true;
    }

    void MinEnclosingTriangleFinderTest::RunTest() {
        area = LinearMinAreaEnclosingTriangleFinder().find(points, triangle);

        cv::convexHull(points, convexHull);
    }

    void MinEnclosingTriangleFinderTest::ValidateTestResults() {
        validationFlag = (
            ArePointsEnclosed() &&
            IsTriangleTouchingPolygon() &&
            IsOneEdgeFlush()
        );
    }

    int MinEnclosingTriangleFinderTest::GetRandomNrOfExecutions() {
        return (MIN_NR_EXECUTIONS + ((double)rand() / numeric_limits<double>::max()) *
                                    (MAX_NR_EXECUTIONS - MIN_NR_EXECUTIONS));
    }

    int MinEnclosingTriangleFinderTest::GetRandomNrOfPoints() {
        return (MIN_NR_POINTS + ((double)rand() / numeric_limits<double>::max()) *
                                (MAX_NR_POINTS - MIN_NR_POINTS));
    }

    bool MinEnclosingTriangleFinderTest::ArePointsEnclosed() {
        double distance = 0;

        for (const Point2d &point : points) {
            distance = pointPolygonTest(triangle, point, true);

            if (distance < -(POINT_IN_TRIANGLE_THRESH)) {
                return false;
            }
        }

        return true;
    }

    bool MinEnclosingTriangleFinderTest::IsTriangleTouchingPolygon() {
        int nrOfPolygonPoints = convexHull.size();

        for (int i = 0; i < 3; i++) {
            bool isTouching = false;
            Point2d middlePoint = Geometry2D::middlePoint(triangle[i], triangle[(i + 1) % 3]);

            for (int j = 0; j < nrOfPolygonPoints; j++) {
                if (Geometry2D::isPointOnLineSegment(middlePoint, convexHull[j],
                                                     convexHull[(j + 1) % nrOfPolygonPoints])) {
                    isTouching = true;
                }
            }

            if (!isTouching) {
                return false;
            }
        }

        return true;
    }

    bool MinEnclosingTriangleFinderTest::IsOneEdgeFlush() {
        int nrOfPolygonPoints = convexHull.size();

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < nrOfPolygonPoints; j++) {
                if ((Geometry2D::isPointOnLineSegment(convexHull[j], triangle[i],
                                                      triangle[(i + 1) % 3])) &&
                    (Geometry2D::isPointOnLineSegment(convexHull[(j + 1) % nrOfPolygonPoints], triangle[i],
                                                      triangle[(i + 1) % 3]))) {
                    return true;
                }
            }
        }

        return false;
    }


    // Constants
    const int MinEnclosingTriangleFinderTest::MIN_NR_POINTS = 1;
    const int MinEnclosingTriangleFinderTest::MAX_NR_POINTS = 10000;
    const int MinEnclosingTriangleFinderTest::MIN_NR_EXECUTIONS = 5000;
    const int MinEnclosingTriangleFinderTest::MAX_NR_EXECUTIONS = 10000;

    const double MinEnclosingTriangleFinderTest::POINT_IN_TRIANGLE_THRESH = 1E-4;

};


// Tests
TEST_F(MinEnclosingTriangleFinderTest, TestNoPoints) {
    EXPECT_THROW(TestNoPoints(), std::exception);
}

TEST_F(MinEnclosingTriangleFinderTest, TestNegativeCoordinates) {
    EXPECT_TRUE(TestPointsWithNegativeXCoordinate());
    EXPECT_TRUE(TestPointsWithNegativeYCoordinate());
    EXPECT_TRUE(TestPointsWithNegativeCoordinates());
}

TEST_F(MinEnclosingTriangleFinderTest, TestVaryingNumberOfPoints) {
    EXPECT_TRUE(TestOnePoint());
    EXPECT_TRUE(TestTwoPoints());
    EXPECT_TRUE(TestThreePoints());
    EXPECT_TRUE(TestMorePoints());
    EXPECT_TRUE(TestMorePointsAndNonEmptyTriangle());
}

TEST_F(MinEnclosingTriangleFinderTest, TestRandomPoints) {
    EXPECT_TRUE(TestRandomPoints());
}


// Main method
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
