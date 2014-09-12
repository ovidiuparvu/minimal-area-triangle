#include "triangle/exception/InvalidInputException.hpp"
#include "triangle/exception/UnexpectedBehaviourException.hpp"
#include "triangle/LinearMinAreaEnclosingTriangleFinder.hpp"
#include "triangle/util/Numeric.hpp"

#include <cassert>
#include <iostream>
#include <limits>

using namespace triangle;
using namespace triangle::util;


LinearMinAreaEnclosingTriangleFinder::LinearMinAreaEnclosingTriangleFinder() {
    validationFlag = 0;

    a = 0;
    b = 0;
    c = 0;
}

LinearMinAreaEnclosingTriangleFinder::~LinearMinAreaEnclosingTriangleFinder() {}

void LinearMinAreaEnclosingTriangleFinder::initialiseAlgorithmVariables() {
    nrOfPoints = polygon.size();

    a = 1;
    b = 2;
    c = 0;
}

void LinearMinAreaEnclosingTriangleFinder::findMinEnclosingTriangle(std::vector<cv::Point2f> &minEnclosingTriangle,
                                                                    double &minEnclosingTriangleArea) {
    for (c = 0; c < nrOfPoints; c++) {
        advanceBToRightChain();
        moveAIfLowAndBIfHigh();
        searchForBTangency();

        updateSidesCA();

        if (isNotBTangency()) {
            updateSidesBA();
        } else {
            updateSideB();
        }

        if (isLocalMinimalTriangle()) {
            updateMinEnclosingTriangle(minEnclosingTriangle, minEnclosingTriangleArea);
        }
    }
}

void LinearMinAreaEnclosingTriangleFinder::advanceBToRightChain() {
    while (Numeric::greaterOrEqual(height(successor(b)), height(b))) {
        advance(b);
    }
}

void LinearMinAreaEnclosingTriangleFinder::moveAIfLowAndBIfHigh() {
    while(height(b) > height(a)) {
        cv::Point2f gammaOfA;

        if ((gamma(a, gammaOfA)) && (intersectsBelow(gammaOfA, b))) {
            advance(b);
        } else {
            advance(a);
        }
    }
}

void LinearMinAreaEnclosingTriangleFinder::searchForBTangency() {
    cv::Point2f gammaOfB;

    while (((gamma(b, gammaOfB)) && (intersectsBelow(gammaOfB, b))) &&
           (Numeric::greaterOrEqual(height(b), height(predecessor(a))))) {
        advance(b);
    }
}

bool LinearMinAreaEnclosingTriangleFinder::isNotBTangency() {
    cv::Point2f gammaOfB;

    if (((gamma(b, gammaOfB)) && (intersectsAbove(gammaOfB, b))) || (height(b) < height(predecessor(a)))) {
        return true;
    }

    return false;
}

void LinearMinAreaEnclosingTriangleFinder::updateSidesCA() {
    sideCStartVertex = polygon[predecessor(c)];
    sideCEndVertex = polygon[c];

    sideAStartVertex = polygon[predecessor(a)];
    sideAEndVertex = polygon[a];
}

void LinearMinAreaEnclosingTriangleFinder::updateSidesBA() {
    // Side B is flush with edge [b, b-1]
    sideBStartVertex = polygon[predecessor(b)];
    sideBEndVertex = polygon[b];

    // Find middle point of side B
    cv::Point2f sideBMiddlePoint;

    if ((middlePointOfSideB(sideBMiddlePoint)) & (height(sideBMiddlePoint) < height(predecessor(a)))) {
        sideAStartVertex = polygon[predecessor(a)];
        sideAEndVertex = findVertexCOnSideB();

        validationFlag = VALIDATION_SIDE_A_TANGENT;
    } else {
        validationFlag = VALIDATION_SIDES_FLUSH;
    }
}

void LinearMinAreaEnclosingTriangleFinder::updateSideB() {
    if (!gamma(b, sideBStartVertex)) {
        MAT_throw(UnexpectedBehaviourException, ERR_SIDE_B_GAMMA);
    }

    sideBEndVertex = polygon[b];

    validationFlag = VALIDATION_SIDE_B_TANGENT;
}

bool LinearMinAreaEnclosingTriangleFinder::isLocalMinimalTriangle() {
    if ((!Geometry2D::lineIntersection(sideAStartVertex, sideAEndVertex, sideBStartVertex, sideBEndVertex, vertexC)) ||
        (!Geometry2D::lineIntersection(sideAStartVertex, sideAEndVertex, sideCStartVertex, sideCEndVertex, vertexB)) ||
        (!Geometry2D::lineIntersection(sideBStartVertex, sideBEndVertex, sideCStartVertex, sideCEndVertex, vertexA))) {
        return false;
    }

    return isValidMinimalTriangle();
}

bool LinearMinAreaEnclosingTriangleFinder::isValidMinimalTriangle() {
    cv::Point2f midpointSideA = Geometry2D::middlePoint(vertexB, vertexC);
    cv::Point2f midpointSideB = Geometry2D::middlePoint(vertexA, vertexC);
    cv::Point2f midpointSideC = Geometry2D::middlePoint(vertexA, vertexB);

    bool sideAValid = (validationFlag == VALIDATION_SIDE_A_TANGENT)
                        ? (Geometry2D::areEqualPoints(midpointSideA, polygon[predecessor(a)]))
                        : (Geometry2D::isPointOnLineSegment(midpointSideA, sideAStartVertex, sideAEndVertex));

    bool sideBValid = (validationFlag == VALIDATION_SIDE_B_TANGENT)
                          ? (Geometry2D::areEqualPoints(midpointSideB, polygon[b]))
                          : (Geometry2D::isPointOnLineSegment(midpointSideB, sideBStartVertex, sideBEndVertex));

    bool sideCValid = Geometry2D::isPointOnLineSegment(midpointSideC, sideCStartVertex, sideCEndVertex);

    return (sideAValid && sideBValid && sideCValid);
}

bool LinearMinAreaEnclosingTriangleFinder::middlePointOfSideB(cv::Point2f &middlePoint) {
    cv::Point2f vertexA, vertexC;

    if ((!Geometry2D::lineIntersection(sideBStartVertex, sideBEndVertex, sideCStartVertex, sideCEndVertex, vertexA)) ||
        (!Geometry2D::lineIntersection(sideBStartVertex, sideBEndVertex, sideAStartVertex, sideAEndVertex, vertexC))) {
        return false;
    }

    middlePoint = Geometry2D::middlePoint(vertexA, vertexC);

    return true;
}

bool LinearMinAreaEnclosingTriangleFinder::intersectsBelow(const cv::Point2f &gammaPoint,
                                                           unsigned int polygonPointIndex) {
    double angleOfGammaAndPoint = Geometry2D::angleOfLineWrtOxAxis(polygon[polygonPointIndex], gammaPoint);

    return (intersects(angleOfGammaAndPoint, polygonPointIndex) == INTERSECTS_BELOW);
}

bool LinearMinAreaEnclosingTriangleFinder::intersectsAbove(const cv::Point2f &gammaPoint,
                                                           unsigned int polygonPointIndex) {
    double angleOfGammaAndPoint = Geometry2D::angleOfLineWrtOxAxis(gammaPoint, polygon[polygonPointIndex]);

    return (intersects(angleOfGammaAndPoint, polygonPointIndex) == INTERSECTS_ABOVE);
}

unsigned int LinearMinAreaEnclosingTriangleFinder::intersects(double angleOfGammaAndPoint,
                                                              unsigned int polygonPointIndex) {
    double angleOfPointAndPredecessor = Geometry2D::angleOfLineWrtOxAxis(polygon[predecessor(polygonPointIndex)],
                                                                         polygon[polygonPointIndex]);
    double angleOfPointAndSuccessor = Geometry2D::angleOfLineWrtOxAxis(polygon[successor(polygonPointIndex)],
                                                                       polygon[polygonPointIndex]);
    double angleOfFlushEdge = Geometry2D::angleOfLineWrtOxAxis(polygon[predecessor(c)], polygon[c]);

    if (isFlushAngleBetweenPredecessorAndSuccessor(angleOfFlushEdge, angleOfPointAndPredecessor, angleOfPointAndSuccessor)) {
        if ((isGammaAngleBetween(angleOfGammaAndPoint, angleOfPointAndPredecessor, angleOfFlushEdge)) ||
            (Numeric::almostEqual(angleOfGammaAndPoint, angleOfPointAndPredecessor))) {
            return intersectsAboveOrBelow(predecessor(polygonPointIndex), polygonPointIndex);
        } else if ((isGammaAngleBetween(angleOfGammaAndPoint, angleOfPointAndSuccessor, angleOfFlushEdge)) ||
                  (Numeric::almostEqual(angleOfGammaAndPoint, angleOfPointAndSuccessor))) {
            return intersectsAboveOrBelow(successor(polygonPointIndex), polygonPointIndex);
        }
    } else {
        if ((isGammaAngleBetween(angleOfGammaAndPoint, angleOfPointAndPredecessor, angleOfPointAndSuccessor)) ||
            ((isGammaAngleEqualTo(angleOfGammaAndPoint, angleOfPointAndPredecessor)) && (!isGammaAngleEqualTo(angleOfGammaAndPoint, angleOfFlushEdge))) ||
            ((isGammaAngleEqualTo(angleOfGammaAndPoint, angleOfPointAndSuccessor)) && (!isGammaAngleEqualTo(angleOfGammaAndPoint, angleOfFlushEdge)))) {
            return INTERSECTS_BELOW;
        }
    }

    return INTERSECTS_CRITICAL;
}

unsigned int LinearMinAreaEnclosingTriangleFinder::intersectsAboveOrBelow(unsigned int successorOrPredecessorIndex,
                                                                          unsigned int pointIndex) {
    if (height(successorOrPredecessorIndex) > height(pointIndex)) {
        return INTERSECTS_ABOVE;
    } else {
        return INTERSECTS_BELOW;
    }
}

bool LinearMinAreaEnclosingTriangleFinder::isFlushAngleBetweenPredecessorAndSuccessor(double &angleFlushEdge,
                                                                                      double anglePredecessor,
                                                                                      double angleSuccessor) {
    if (Geometry2D::isAngleBetweenNonReflex(angleFlushEdge, anglePredecessor, angleSuccessor)) {
        return true;
    } else if (Geometry2D::isOppositeAngleBetweenNonReflex(angleFlushEdge, anglePredecessor, angleSuccessor)) {
        angleFlushEdge = Geometry2D::oppositeAngle(angleFlushEdge);

        return true;
    }

    return false;
}

bool LinearMinAreaEnclosingTriangleFinder::isGammaAngleBetween(double &gammaAngle, double angle1, double angle2) {
    return (Geometry2D::isAngleBetweenNonReflex(gammaAngle, angle1, angle2));
}

bool LinearMinAreaEnclosingTriangleFinder::isGammaAngleEqualTo(double &gammaAngle, double angle) {
    return (Numeric::almostEqual(gammaAngle, angle));
}

double LinearMinAreaEnclosingTriangleFinder::height(unsigned int polygonPointIndex) {
    cv::Point2f pointC = polygon[c];
    cv::Point2f pointCPredecessor = polygon[predecessor(c)];

    cv::Point2f polygonPoint = polygon[polygonPointIndex];

    return Geometry2D::distanceFromPointToLine(polygonPoint, pointC, pointCPredecessor);
}

double LinearMinAreaEnclosingTriangleFinder::height(const cv::Point2f &polygonPoint) {
    cv::Point2f pointC = polygon[c];
    cv::Point2f pointCPredecessor = polygon[predecessor(c)];

    return Geometry2D::distanceFromPointToLine(polygonPoint, pointC, pointCPredecessor);
}

bool LinearMinAreaEnclosingTriangleFinder::gamma(unsigned int polygonPointIndex, cv::Point2f &gammaPoint) {
    cv::Point2f intersectionPoint1, intersectionPoint2;

    // Get intersection points if they exist
    if (!findGammaIntersectionPoints(polygonPointIndex, polygon[a], polygon[predecessor(a)], polygon[c],
                                     polygon[predecessor(c)], intersectionPoint1, intersectionPoint2)) {
        return false;
    }

    // Select the point which is on the same side of line C as the polygon
    if (Geometry2D::areOnTheSameSideOfLine(intersectionPoint1, polygon[successor(c)],
                                           polygon[c], polygon[predecessor(c)])) {
        gammaPoint = intersectionPoint1;
    } else {
        gammaPoint = intersectionPoint2;
    }

    return true;
}

cv::Point2f LinearMinAreaEnclosingTriangleFinder::findVertexCOnSideB() {
    cv::Point2f intersectionPoint1, intersectionPoint2;

    // Get intersection points if they exist
    if (!findGammaIntersectionPoints(predecessor(a), sideBStartVertex, sideBEndVertex, sideCStartVertex,
                                     sideCEndVertex, intersectionPoint1, intersectionPoint2)) {
        MAT_throw(UnexpectedBehaviourException, ERR_VERTEX_C_ON_SIDE_B);
    }

    // Select the point which is on the same side of line C as the polygon
    if (Geometry2D::areOnTheSameSideOfLine(intersectionPoint1, polygon[successor(c)],
                                           polygon[c], polygon[predecessor(c)])) {
        return intersectionPoint1;
    } else {
        return intersectionPoint2;
    }
}

bool LinearMinAreaEnclosingTriangleFinder::findGammaIntersectionPoints(unsigned int polygonPointIndex,
                                                                       const cv::Point2f &side1StartVertex,
                                                                       const cv::Point2f &side1EndVertex,
                                                                       const cv::Point2f &side2StartVertex,
                                                                       const cv::Point2f &side2EndVertex,
                                                                       cv::Point2f &intersectionPoint1,
                                                                       cv::Point2f &intersectionPoint2) {
    std::vector<double> side1Params = lineEquationParameters(side1StartVertex, side1EndVertex);
    std::vector<double> side2Params = lineEquationParameters(side2StartVertex, side2EndVertex);

    // Compute side C extra parameter using the formula for distance from a point to a line
    double polygonPointHeight = height(polygonPointIndex);
    double distanceFormulaDenominator = sqrt((side2Params[0] * side2Params[0]) + (side2Params[1] * side2Params[1]));
    double sideCExtraParam = 2 * polygonPointHeight * distanceFormulaDenominator;

    // Get intersection points if they exist or if lines are identical
    if (!areIntersectingLines(side1Params, side2Params, sideCExtraParam, intersectionPoint1, intersectionPoint2)) {
        return false;
    } else if (areIdenticalLines(side1Params, side2Params, sideCExtraParam)) {
        intersectionPoint1 = side1StartVertex;
        intersectionPoint2 = side1EndVertex;
    }

    return true;
}


// Constants
const unsigned int LinearMinAreaEnclosingTriangleFinder::INTERSECTS_BELOW      = 1;
const unsigned int LinearMinAreaEnclosingTriangleFinder::INTERSECTS_ABOVE      = 2;
const unsigned int LinearMinAreaEnclosingTriangleFinder::INTERSECTS_CRITICAL   = 3;
const unsigned int LinearMinAreaEnclosingTriangleFinder::INTERSECTS_LIMIT      = 4;

const std::string LinearMinAreaEnclosingTriangleFinder::ERR_MIDPOINT_SIDE_B     = "The position of the middle point of side B could not be determined.";
const std::string LinearMinAreaEnclosingTriangleFinder::ERR_SIDE_B_GAMMA        = "The position of side B could not be determined, because gamma(b) could not be computed.";
const std::string LinearMinAreaEnclosingTriangleFinder::ERR_VERTEX_C_ON_SIDE_B  = "The position of the vertex C on side B could not be determined, because the considered lines do not intersect.";

const unsigned int LinearMinAreaEnclosingTriangleFinder::VALIDATION_SIDE_A_TANGENT   = 0;
const unsigned int LinearMinAreaEnclosingTriangleFinder::VALIDATION_SIDE_B_TANGENT   = 1;
const unsigned int LinearMinAreaEnclosingTriangleFinder::VALIDATION_SIDES_FLUSH      = 2;
