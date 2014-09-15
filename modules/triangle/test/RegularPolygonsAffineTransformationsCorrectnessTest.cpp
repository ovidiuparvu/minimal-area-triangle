#include "triangle/LinearMinAreaEnclosingTriangleFinder.hpp"
#include "triangle/exception/ExceptionHandler.hpp"
#include "triangle/exception/InvalidInputException.hpp"
#include "triangle/exception/UnexpectedBehaviourException.hpp"
#include "triangle/util/StringManipulator.hpp"

#include <ctime>
#include <fstream>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

using namespace triangle;
using namespace triangle::util;

const std::string ERR_DIFFERENT_TRIANGLE_AREA_BEGIN = "The area of the enclosing and expected triangles differs considering the regular polygon defined by ";
const std::string ERR_DIFFERENT_TRIANGLE_AREA_END   = " points.";

const double POLYGON_SIDE_LENGTH = 1000;

const double POLYGON_ORIGIN_X = 2000;
const double POLYGON_ORIGIN_Y = 2000;

const float SCALING_FACTOR_X = 1.5;
const float SCALING_FACTOR_Y = 1.5;

const double ROTATION_ANGLE = (Geometry2D::PI / 4);

const unsigned int NR_POLYGON_POINTS = 10002;


// Compute the expected area for the equilateral triangle defined by the given number of points
/* The area of the triangle is equal to l^2 * sqrt(3) / 4, where
 *     l = POLYGON_SIDE_LENGTH + 2 * l',
 * respectively
 *     l' = 2 * r * sin(alpha)
 * and
 *     alpha = (pi / 3n) * (n - 3).
 */
double computeExpectedTriangleArea(unsigned int nrOfPoints) {
    double alpha  = (Geometry2D::PI / (3.0 * nrOfPoints)) * (nrOfPoints - 3.0);
    double radius = POLYGON_SIDE_LENGTH / (2.0 * std::sin(Geometry2D::PI / nrOfPoints));

    double triangleSideSubSegment   = 2 * radius * std::sin(alpha);
    double triangleSideLength       = POLYGON_SIDE_LENGTH + (2 * triangleSideSubSegment);

    return (
        (std::pow(triangleSideLength, 2) * sqrt(3) / 4) *
        SCALING_FACTOR_X * SCALING_FACTOR_Y
    );
}

// Apply the scaling affine transformation to the given point
void applyScalingAffineTransformation(cv::Point2f &polygonPoint) {
    polygonPoint.x = polygonPoint.x * SCALING_FACTOR_X;
    polygonPoint.y = polygonPoint.y * SCALING_FACTOR_Y;
}

// Apply the rotation affine transformation to the given point
void applyRotationAffineTransformation(cv::Point2f &polygonPoint) {
    // Translation
    polygonPoint.x = polygonPoint.x - POLYGON_ORIGIN_X;
    polygonPoint.y = polygonPoint.y - POLYGON_ORIGIN_Y;

    // Store old polygon point coordinates
    float oldPolygonPointXCoord = polygonPoint.x;
    float oldPolygonPointYCoord = polygonPoint.y;

    // Rotation
    polygonPoint.x = (oldPolygonPointXCoord * std::cos(ROTATION_ANGLE)) +
                     (oldPolygonPointYCoord * std::sin(ROTATION_ANGLE));
    polygonPoint.y = -(oldPolygonPointXCoord * std::sin(ROTATION_ANGLE)) +
                      (oldPolygonPointYCoord * std::cos(ROTATION_ANGLE));

    // Translation back
    polygonPoint.x = polygonPoint.x + POLYGON_ORIGIN_X;
    polygonPoint.y = polygonPoint.y + POLYGON_ORIGIN_Y;
}

// Return the resulting polygon after applying the rotation and scaling affine transformations
void applyAffineTransformations(std::vector<cv::Point2f> &regularPolygon) {
    for (cv::Point2f &polygonPoint : regularPolygon) {
        applyRotationAffineTransformation(polygonPoint);
        applyScalingAffineTransformation(polygonPoint);
    }
}

// Generate a regular polygon comprising the specified number of points and considering the given arc angle and radius
std::vector<cv::Point2f> generateRegularPolygon(unsigned int nrOfPoints, double arcAngle, double radius) {
    std::vector<cv::Point2f> regularPolygon;

    // Add points to regular polygon
    for (unsigned int i = 0; i < nrOfPoints; i++) {
        regularPolygon.push_back(
            cv::Point2f(
                POLYGON_ORIGIN_X + (radius * std::cos(i * arcAngle)),
                POLYGON_ORIGIN_Y + (radius * std::sin(i * arcAngle))
            )
        );
    }

    // Apply the affine transformations to the generated regular polygon
    applyAffineTransformations(regularPolygon);

    return regularPolygon;
}

// Generate a regular polygon comprising the specified number of points
std::vector<cv::Point2f> generateRegularPolygon(unsigned int nrOfPoints) {
    double arcAngle = (Geometry2D::PI * 2.0) / static_cast<double>(nrOfPoints);
    double radius   = POLYGON_SIDE_LENGTH / (2.0 * std::sin(arcAngle / 2.0));

    return generateRegularPolygon(nrOfPoints, arcAngle, radius);
}

// Compute the area of the minimal triangle enclosing a polygon defined by the given number of points
double computeEnclosingTriangleArea(unsigned int nrOfPoints) {
    std::vector<cv::Point2f> minimalEnclosingTriangle;

    std::vector<cv::Point2f> regularPolygon = generateRegularPolygon(nrOfPoints);

    return LinearMinAreaEnclosingTriangleFinder().find(regularPolygon, minimalEnclosingTriangle);
}

// Run the regular polygons considering affine transformations correctness test for a polygon with the
// specified number of points
void runRegularPolygonAffineTransformationCorrectnessTest(unsigned int nrOfPoints) {
    double enclosingTriangleArea = computeEnclosingTriangleArea(nrOfPoints);
    double expectedTriangleArea  = computeExpectedTriangleArea(nrOfPoints);

    if (!Numeric::almostEqual(enclosingTriangleArea, expectedTriangleArea)) {
        MAT_throw(
            UnexpectedBehaviourException,
            ERR_DIFFERENT_TRIANGLE_AREA_BEGIN +
            StringManipulator::toString<unsigned int>(nrOfPoints) +
            ERR_DIFFERENT_TRIANGLE_AREA_END
        );
    }
}

// Run the regular polygons considering affine transformations correctness test
void runRegularPolygonsAffineTransformationCorrectnessTest() {
    for (unsigned int i = 3; i <= NR_POLYGON_POINTS; i += 3) {
        // Inform the user which k-gon is tested next
        std::cout << "Running the correctness test for the regular " << i << "-gon to which affine transformations (scaling and rotation) were applied..." << std::endl;

        // Run the correctness test
        runRegularPolygonAffineTransformationCorrectnessTest(i);
    }

    // Inform the user that all tests were executed successfully
    std::cout << "[ SUCCESS ] All tests executed successfully." << std::endl;
}

// Main function
int main() {
    try {
        runRegularPolygonsAffineTransformationCorrectnessTest();

        return EXEC_SUCCESS_CODE;
    } catch (const MinimalAreaTriangleException &ex) {
        ExceptionHandler::printErrorMessage(ex);

        return EXEC_ERR_CODE;
    }
}
