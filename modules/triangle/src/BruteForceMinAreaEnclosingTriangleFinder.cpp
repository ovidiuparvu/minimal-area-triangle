#include "triangle/exception/InvalidInputException.hpp"
#include "triangle/exception/MinimalAreaTriangleException.hpp"
#include "triangle/BruteForceMinAreaEnclosingTriangleFinder.hpp"
#include "triangle/util/Geometry2D.hpp"
#include "triangle/util/Numeric.hpp"

#include <cassert>
#include <iostream>
#include <limits>

using namespace triangle;
using namespace triangle::util;


BruteForceMinAreaEnclosingTriangleFinder::BruteForceMinAreaEnclosingTriangleFinder() {
    c = 0;
    a = 0;
    b = 0;
}

BruteForceMinAreaEnclosingTriangleFinder::~BruteForceMinAreaEnclosingTriangleFinder() {}

void
BruteForceMinAreaEnclosingTriangleFinder::initialiseAlgorithmVariables() {
    nrOfPoints = polygon.size();
}

void
BruteForceMinAreaEnclosingTriangleFinder::findMinEnclosingTriangle(std::vector<cv::Point2f> &minEnclosingTriangle,
                                                                   double &minEnclosingTriangleArea) {
    for (c = 0; c < nrOfPoints; c++) {
        for (a = 0; a < nrOfPoints; a++) {
            if (areValidIntersectingLines(predecessor(a), a, predecessor(c), c)) {
                for (b = 0; b < nrOfPoints; b++) {
                    if (areValidPolygonPoints(c, a, b)) {
                        findMinEnclosingTriangleTangentSideB(minEnclosingTriangle, minEnclosingTriangleArea);
                        findMinEnclosingTriangleFlushSideB(minEnclosingTriangle, minEnclosingTriangleArea);
                    }
                }
            }
        }
    }
}

bool
BruteForceMinAreaEnclosingTriangleFinder::areValidIntersectingLines(unsigned int firstLineFirstIndex,
                                                                    unsigned int firstLineSecondIndex,
                                                                    unsigned int secondLineFirstIndex,
                                                                    unsigned int secondLineSecondIndex) {
    if ((firstLineFirstIndex != secondLineFirstIndex) ||
        (firstLineSecondIndex != secondLineSecondIndex)) {
        return (
            Geometry2D::lineIntersection(
                polygon[firstLineFirstIndex], polygon[firstLineSecondIndex],
                polygon[secondLineFirstIndex], polygon[secondLineSecondIndex],
                vertexB
            )
        );
    }

    return false;
}

bool
BruteForceMinAreaEnclosingTriangleFinder::areValidPolygonPoints(unsigned int firstPolygonPointIndex,
                                                                unsigned int secondPolygonPointIndex,
                                                                unsigned int thirdPolygonPointIndex) {
    return (
        (firstPolygonPointIndex != secondPolygonPointIndex) &&
        (firstPolygonPointIndex != thirdPolygonPointIndex) &&
        (secondPolygonPointIndex != thirdPolygonPointIndex)
    );
}

void
BruteForceMinAreaEnclosingTriangleFinder::findMinEnclosingTriangleTangentSideB(std::vector<cv::Point2f>
                                                                               &minEnclosingTriangle,
                                                                               double &minEnclosingTriangleArea) {
    if ((b != predecessor(c)) && (b != predecessor(a))) {
        std::vector<double> sideCParameters = lineEquationParameters(polygon[predecessor(c)], polygon[c]);
        std::vector<double> sideAParameters = lineEquationParameters(polygon[predecessor(a)], polygon[a]);

        findMinEnclosingTriangleTangentSideB(sideCParameters, sideAParameters, minEnclosingTriangle,
                                             minEnclosingTriangleArea);
    }
}

void
BruteForceMinAreaEnclosingTriangleFinder::findMinEnclosingTriangleTangentSideB(const std::vector<double>
                                                                               &sideCParameters,
                                                                               const std::vector<double>
                                                                               &sideAParameters,
                                                                               std::vector<cv::Point2f>
                                                                               &minEnclosingTriangle,
                                                                               double &minEnclosingTriangleArea) {
    if (areParallelLines(sideCParameters, sideAParameters)) {
        MAT_throw(InvalidInputException, ERR_PARALLEL_TRIANGLE_SIDES);
    } else {
        updateVerticesCAndA(sideCParameters, sideAParameters);

        if (isValidTangentSideB()) {
            computeEnclosingTriangle(minEnclosingTriangle, minEnclosingTriangleArea);
        }
    }
}

void
BruteForceMinAreaEnclosingTriangleFinder::findMinEnclosingTriangleFlushSideB(std::vector<cv::Point2f>
                                                                             &minEnclosingTriangle,
                                                                             double &minEnclosingTriangleArea) {
    // If vertices A and C exist
    if ((Geometry2D::lineIntersection(polygon[predecessor(b)], polygon[b],
                                      polygon[predecessor(c)], polygon[c], vertexA)) &&
        (Geometry2D::lineIntersection(polygon[predecessor(a)], polygon[a],
                                      polygon[predecessor(b)], polygon[b], vertexC))) {
        computeEnclosingTriangle(minEnclosingTriangle, minEnclosingTriangleArea);
    }
}

bool
BruteForceMinAreaEnclosingTriangleFinder::areParallelLines(const std::vector<double> &sideCParameters,
                                                           const std::vector<double> &sideAParameters) {
    double determinant = (sideCParameters[0] * sideAParameters[1]) -
                         (sideAParameters[0] * sideCParameters[1]);

    return (Numeric::almostEqual(determinant, 0));
}

void
BruteForceMinAreaEnclosingTriangleFinder::updateVerticesCAndA(const std::vector<double> &sideCParameters,
                                                              const std::vector<double> &sideAParameters) {
    // Side A parameters
    double a1 = sideCParameters[0];
    double b1 = sideCParameters[1];
    double c1 = sideCParameters[2];

    // Side B parameters
    double a2 = sideAParameters[0];
    double b2 = sideAParameters[1];
    double c2 = sideAParameters[2];

    // Polygon point "b" coordinates
    double m = polygon[b].x;
    double n = polygon[b].y;

    // Compute vertices A and C x-coordinates
    double x2 = ((2 * b1 * b2 * n) + (c1 * b2) + (2 * a1 * b2 * m) + (b1 * c2)) / ((a1 * b2) - (a2 * b1));
    double x1 = (2 * m) - x2;

    // Compute vertices A and C y-coordinates
    double y2 = 0;
    double y1 = 0;

    if (Numeric::almostEqual(b1, 0)) {          // b1 = 0 and b2 != 0
        y2 = ((-c2) - (a2 * x2)) / (b2);
        y1 = (2 * n) - y2;
    } else if (Numeric::almostEqual(b2, 0)) {   // b1 != 0 and b2 = 0
        y1 = ((-c1) - (a1 * x1)) / (b1);
        y2 = (2 * n) - y1;
    } else {                                    // b1 != 0 and b2 != 0
        y1 = ((-c1) - (a1 * x1)) / (b1);
        y2 = ((-c2) - (a2 * x2)) / (b2);
    }

    // Update vertices A and C coordinates
    vertexA.x = x1;
    vertexA.y = y1;
    vertexC.x = x2;
    vertexC.y = y2;
}

bool
BruteForceMinAreaEnclosingTriangleFinder::isValidTangentSideB() {
    double angleOfTangentSideB  = Geometry2D::angleOfLineWrtOxAxis(vertexC, vertexA);
    double anglePredecessor     = Geometry2D::angleOfLineWrtOxAxis(polygon[predecessor(b)], polygon[b]);
    double angleSuccessor       = Geometry2D::angleOfLineWrtOxAxis(polygon[b], polygon[successor(b)]);

    return (
        (Geometry2D::isAngleBetweenNonReflex(angleOfTangentSideB, anglePredecessor, angleSuccessor)) ||
        (Geometry2D::isOppositeAngleBetweenNonReflex(angleOfTangentSideB, anglePredecessor, angleSuccessor))
    );
}

void
BruteForceMinAreaEnclosingTriangleFinder::computeEnclosingTriangle(std::vector<cv::Point2f> &minEnclosingTriangle,
                                                                   double &minEnclosingTriangleArea) {
    if (isValidMinimalTriangle()) {
        updateMinEnclosingTriangle(minEnclosingTriangle, minEnclosingTriangleArea);
    }
}

bool
BruteForceMinAreaEnclosingTriangleFinder::isValidMinimalTriangle() {
    std::vector<cv::Point2f> currentMinEnclosingTriangle({vertexA, vertexB, vertexC});

    // Check if all polygon points are contained by the triangle
    for (unsigned int i = 0; i < nrOfPoints; i++) {
        double distance = cv::pointPolygonTest(currentMinEnclosingTriangle, polygon[i], true);

        if (distance < -(POLYGON_POINT_TEST_THRESHOLD)) {
            return false;
        }
    }

    return true;
}


// Constants
const std::string BruteForceMinAreaEnclosingTriangleFinder::ERR_PARALLEL_TRIANGLE_SIDES = "At least two sides of the triangle are parallel when they should be intersecting. Please change.";

const double BruteForceMinAreaEnclosingTriangleFinder::POLYGON_POINT_TEST_THRESHOLD = 1E-4;
