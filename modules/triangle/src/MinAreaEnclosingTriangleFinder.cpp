#include "triangle/exception/InvalidInputException.hpp"
#include "triangle/exception/UnexpectedBehaviourException.hpp"
#include "triangle/MinAreaEnclosingTriangleFinder.hpp"
#include "triangle/util/Numeric.hpp"

#include <cassert>
#include <iostream>
#include <limits>

using namespace triangle;
using namespace triangle::util;


MinAreaEnclosingTriangleFinder::MinAreaEnclosingTriangleFinder() {
    area = 0;

    nrOfPoints = 0;
}

MinAreaEnclosingTriangleFinder::~MinAreaEnclosingTriangleFinder() {}

double MinAreaEnclosingTriangleFinder::find(const std::vector<cv::Point2f> &points,
                                            std::vector<cv::Point2f> &minEnclosingTriangle) {
    if (points.size() == 0) {
        MAT_throw(InvalidInputException, ERR_NR_POINTS);

        // Added to overcome warning messages
        throw InvalidInputException(__FILE__, __LINE__, ERR_NR_POINTS);
    } else {
        return findMinTriangle(points, minEnclosingTriangle);
    }
}

double MinAreaEnclosingTriangleFinder::findMinTriangle(const std::vector<cv::Point2f> &points,
                                                       std::vector<cv::Point2f> &minEnclosingTriangle) {
    initialise(points, minEnclosingTriangle);

    if (polygon.size() > 3) {
        return findMinEnclosingTriangle(polygon, minEnclosingTriangle);
    } else {
        return returnMinEnclosingTriangle(polygon, minEnclosingTriangle);
    }
}

void MinAreaEnclosingTriangleFinder::initialise(const std::vector<cv::Point2f> &points,
                                                std::vector<cv::Point2f> &minEnclosingTriangle) {
    // Clear all points previously stored in the std::vector
    minEnclosingTriangle.clear();

    initialiseConvexPolygon(points);
}

void MinAreaEnclosingTriangleFinder::initialiseConvexPolygon(const std::vector<cv::Point2f> &points) {
    polygon.clear();

    cv::convexHull(points, polygon, CONVEX_HULL_CLOCKWISE);
}

double MinAreaEnclosingTriangleFinder::findMinEnclosingTriangle(const std::vector<cv::Point2f> &polygon,
                                                                std::vector<cv::Point2f> &minEnclosingTriangle) {
    double minEnclosingTriangleArea = std::numeric_limits<double>::max();

    initialiseAlgorithmVariables();

    findMinEnclosingTriangle(minEnclosingTriangle, minEnclosingTriangleArea);

    return minEnclosingTriangleArea;
}

double MinAreaEnclosingTriangleFinder::returnMinEnclosingTriangle(const std::vector<cv::Point2f> &polygon,
                                                              std::vector<cv::Point2f> &minEnclosingTriangle) {
    int nrOfPolygonPoints = polygon.size();

    for (int i = 0; i < 3; i++) {
        minEnclosingTriangle.push_back(polygon[i % nrOfPolygonPoints]);
    }

    return Geometry2D::areaOfTriangle(minEnclosingTriangle[0], minEnclosingTriangle[1], minEnclosingTriangle[2]);
}

void MinAreaEnclosingTriangleFinder::updateMinEnclosingTriangle(std::vector<cv::Point2f> &minEnclosingTriangle, double &minEnclosingTriangleArea) {
    area = Geometry2D::areaOfTriangle(vertexA, vertexB, vertexC);

    if (area < minEnclosingTriangleArea) {
        minEnclosingTriangle.clear();

        minEnclosingTriangle.push_back(vertexA);
        minEnclosingTriangle.push_back(vertexB);
        minEnclosingTriangle.push_back(vertexC);

        minEnclosingTriangleArea = area;
    }
}

bool MinAreaEnclosingTriangleFinder::areIdenticalLines(const std::vector<double> &side1Params,
                                                       const std::vector<double> &side2Params,
                                                       double sideCExtraParam) {
    return (
        (Geometry2D::areIdenticalLines(side1Params[0], side1Params[1], -(side1Params[2]),
                                       side2Params[0], side2Params[1], -(side2Params[2]) - sideCExtraParam)) ||
        (Geometry2D::areIdenticalLines(side1Params[0], side1Params[1], -(side1Params[2]),
                                       side2Params[0], side2Params[1], -(side2Params[2]) + sideCExtraParam))
    );
}

bool MinAreaEnclosingTriangleFinder::areIntersectingLines(const std::vector<double> &side1Params,
                                                          const std::vector<double> &side2Params,
                                                          double sideCExtraParam,
                                                          cv::Point2f &intersectionPoint1,
                                                          cv::Point2f &intersectionPoint2) {
    return (
        (Geometry2D::lineIntersection(side1Params[0], side1Params[1], -(side1Params[2]),
                                      side2Params[0], side2Params[1], -(side2Params[2]) - sideCExtraParam,
                                      intersectionPoint1)) &&
        (Geometry2D::lineIntersection(side1Params[0], side1Params[1], -(side1Params[2]),
                                      side2Params[0], side2Params[1], -(side2Params[2]) + sideCExtraParam,
                                      intersectionPoint2))
    );
}

std::vector<double> MinAreaEnclosingTriangleFinder::lineEquationParameters(const cv::Point2f &p,
                                                                           const cv::Point2f &q) {
    std::vector<double> lineEquationParameters;
    double a, b, c;

    Geometry2D::lineEquationDeterminedByPoints(p, q, a, b, c);

    lineEquationParameters.push_back(a);
    lineEquationParameters.push_back(b);
    lineEquationParameters.push_back(c);

    return lineEquationParameters;
}

void MinAreaEnclosingTriangleFinder::advance(unsigned int &index) {
    index = successor(index);
}

unsigned int MinAreaEnclosingTriangleFinder::successor(unsigned int index) {
    return ((index + 1) % nrOfPoints);
}

unsigned int MinAreaEnclosingTriangleFinder::predecessor(unsigned int index) {
    return (index == 0) ? (nrOfPoints - 1)
                        : (index - 1);
}


// Constants
const bool MinAreaEnclosingTriangleFinder::CONVEX_HULL_CLOCKWISE = true;

const std::string MinAreaEnclosingTriangleFinder::ERR_NR_POINTS           = "The number of 2D points in the input vector should be greater than 0.";
const std::string MinAreaEnclosingTriangleFinder::ERR_TRIANGLE_VERTICES   = "The position of the triangle vertices could not be determined, because the sides of the triangle do not intersect.";
