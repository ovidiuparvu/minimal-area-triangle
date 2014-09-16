#ifndef MINAREAENCLOSINGTRIANGLEFINDERTEST_HPP
#define MINAREAENCLOSINGTRIANGLEFINDERTEST_HPP

#include "triangle/core/MinimalAreaTriangleTest.hpp"
#include "triangle/LinearMinAreaEnclosingTriangleFinder.hpp"

#include <limits>
#include <vector>

using namespace triangle;
using namespace triangle::util;
using namespace triangletest;


namespace triangletest {

    //! Class for testing the minimum enclosing triangle algorithm
    class MinAreaEnclosingTriangleFinderTest : public MinimalAreaTriangleTest {

        protected:

            std::vector<cv::Point2f> convexHull;    /*!< Convex hull of the 2D point set */

            std::vector<cv::Point2f> points;        /*!< Collection of 2D points */
            std::vector<cv::Point2f> triangle;      /*!< Minimum enclosing triangle */
            double area;                            /*!< Area of the minimum enclosing triangle */

        public:

           MinAreaEnclosingTriangleFinderTest();
           virtual ~MinAreaEnclosingTriangleFinderTest();

           //! Test the scenario when an empty std::vector of points is provided
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

           //! Test the scenario when the output std::vector is not empty
           bool TestMorePointsAndNonEmptyTriangle();

           //! Test the scenario when randomly initialised std::vectors of input points are provided
           bool TestRandomPoints();

        protected:

           //! Run the test for the given set of points
           virtual void RunTest() = 0;

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

    MinAreaEnclosingTriangleFinderTest::MinAreaEnclosingTriangleFinderTest() {
        area = 0;

        points.clear();
        convexHull.clear();
        triangle.clear();
    }

    MinAreaEnclosingTriangleFinderTest::~MinAreaEnclosingTriangleFinderTest() {
        area = 0;

        points.clear();
        convexHull.clear();
        triangle.clear();
    }

    bool MinAreaEnclosingTriangleFinderTest::TestNoPoints() {
        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinAreaEnclosingTriangleFinderTest::TestPointsWithNegativeXCoordinate() {
        points = std::vector<cv::Point2f>({cv::Point2f(-2, 1), cv::Point2f(2, 2), cv::Point2f(4, 4)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinAreaEnclosingTriangleFinderTest::TestPointsWithNegativeYCoordinate() {
        points = std::vector<cv::Point2f>({cv::Point2f(2, -1), cv::Point2f(2, 2), cv::Point2f(4, 4)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinAreaEnclosingTriangleFinderTest::TestPointsWithNegativeCoordinates() {
        points = std::vector<cv::Point2f>({cv::Point2f(-2, -1), cv::Point2f(2, 2), cv::Point2f(4, 4)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinAreaEnclosingTriangleFinderTest::TestOnePoint() {
        points = std::vector<cv::Point2f>({cv::Point2f(1, 1)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinAreaEnclosingTriangleFinderTest::TestTwoPoints() {
        points = std::vector<cv::Point2f>({cv::Point2f(1, 1), cv::Point2f(2, 2)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinAreaEnclosingTriangleFinderTest::TestThreePoints() {
        points = std::vector<cv::Point2f>({cv::Point2f(1, 1), cv::Point2f(2, 2), cv::Point2f(4, 1)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinAreaEnclosingTriangleFinderTest::TestMorePoints() {
        points = std::vector<cv::Point2f>({cv::Point2f(1, 1), cv::Point2f(2, 2), cv::Point2f(4, 1), cv::Point2f(5, 5), cv::Point2f(6, 4)});

        RunTest();
        ValidateTestResults();

        return validationFlag;
    }

    bool MinAreaEnclosingTriangleFinderTest::TestMorePointsAndNonEmptyTriangle() {
        points = std::vector<cv::Point2f>({cv::Point2f(1, 1), cv::Point2f(2, 2), cv::Point2f(4, 1), cv::Point2f(5, 5), cv::Point2f(6, 4)});

        triangle.push_back(cv::Point2f(1, 1));

        RunTest();
        ValidateTestResults();

        return ((validationFlag) && (triangle.size() == 3));
    }

    bool MinAreaEnclosingTriangleFinderTest::TestRandomPoints() {
        srand(time(NULL));

        int nrOfRandomTrials = GetRandomNrOfExecutions();

        for (int i = 0; i < nrOfRandomTrials; i++) {
            points.clear();

            int nrOfPoints = GetRandomNrOfPoints();

            for (int j = 0; j < nrOfPoints; j++) {
                points.push_back(
                    cv::Point2f(
                        static_cast<float>(rand()),
                        static_cast<float>(rand())
                    )
                );
            }

            RunTest();
            ValidateTestResults();

            if (!validationFlag) {
                return false;
            }
        }

        return true;
    }

    void MinAreaEnclosingTriangleFinderTest::ValidateTestResults() {
        validationFlag = (
            ArePointsEnclosed() &&
            IsTriangleTouchingPolygon() &&
            IsOneEdgeFlush()
        );
    }

    int MinAreaEnclosingTriangleFinderTest::GetRandomNrOfExecutions() {
        return (
            MIN_NR_EXECUTIONS +
            static_cast<int>(
                (static_cast<double>(rand()) / std::numeric_limits<double>::max()) *
                static_cast<double>(MAX_NR_EXECUTIONS - MIN_NR_EXECUTIONS)
            )
        );
    }

    int MinAreaEnclosingTriangleFinderTest::GetRandomNrOfPoints() {
        return (
            MIN_NR_POINTS +
            static_cast<int>(
                (static_cast<double>(rand()) / std::numeric_limits<double>::max()) *
                static_cast<double>(MAX_NR_POINTS - MIN_NR_POINTS)
            )
        );
    }

    bool MinAreaEnclosingTriangleFinderTest::ArePointsEnclosed() {
        double distance = 0;

        for (const cv::Point2f &point : points) {
            distance = pointPolygonTest(triangle, point, true);

            if (distance < -(POINT_IN_TRIANGLE_THRESH)) {
                return false;
            }
        }

        return true;
    }

    bool MinAreaEnclosingTriangleFinderTest::IsTriangleTouchingPolygon() {
        std::size_t nrOfPolygonPoints = convexHull.size();

        for (std::size_t i = 0; i < 3; i++) {
            bool isTouching = false;
            cv::Point2f middlePoint = Geometry2D::middlePoint(triangle[i], triangle[(i + 1) % 3]);

            for (std::size_t j = 0; j < nrOfPolygonPoints; j++) {
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

    bool MinAreaEnclosingTriangleFinderTest::IsOneEdgeFlush() {
        std::size_t nrOfPolygonPoints = convexHull.size();

        for (std::size_t i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < nrOfPolygonPoints; j++) {
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
    const int MinAreaEnclosingTriangleFinderTest::MIN_NR_POINTS = 1;
    const int MinAreaEnclosingTriangleFinderTest::MAX_NR_POINTS = 10000;
    const int MinAreaEnclosingTriangleFinderTest::MIN_NR_EXECUTIONS = 5000;
    const int MinAreaEnclosingTriangleFinderTest::MAX_NR_EXECUTIONS = 10000;

    const double MinAreaEnclosingTriangleFinderTest::POINT_IN_TRIANGLE_THRESH = 1E-4;

};


#endif
