#include "triangle/exception/InvalidInputException.hpp"
#include "triangle/exception/UnexpectedBehaviourException.hpp"
#include "triangle/LinearMinAreaEnclosingTriangleFinder.hpp"
#include "triangle/Numeric.hpp"

#include <cassert>
#include <iostream>
#include <limits>

using namespace triangle;


LinearMinAreaEnclosingTriangleFinder::LinearMinAreaEnclosingTriangleFinder() {
    validationFlag = 0;

    area = 0;

    a = 0;
    b = 0;
    c = 0;

    nrOfPoints = 0;
}

LinearMinAreaEnclosingTriangleFinder::~LinearMinAreaEnclosingTriangleFinder() {}

double LinearMinAreaEnclosingTriangleFinder::find(const vector<Point2d> &points, vector<Point2d> &minEnclosingTriangle) {
    if (points.size() == 0) {
        MAT_throw(InvalidInputException, ERR_NR_POINTS);

        // Added to overcome warning messages
        throw InvalidInputException(__FILE__, __LINE__, ERR_NR_POINTS);
    } else {
        return findMinTriangle(points, minEnclosingTriangle);
    }
}

double LinearMinAreaEnclosingTriangleFinder::findMinTriangle(const vector<Point2d> &points, vector<Point2d> &minEnclosingTriangle) {
    initialise(points, minEnclosingTriangle);

    if (polygon.size() > 3) {
        return findMinEnclosingTriangle(polygon, minEnclosingTriangle);
    } else {
        return returnMinEnclosingTriangle(polygon, minEnclosingTriangle);
    }
}

void LinearMinAreaEnclosingTriangleFinder::initialise(const vector<Point2d> &points, vector<Point2d> &minEnclosingTriangle) {
    // Clear all points previously stored in the vector
    minEnclosingTriangle.clear();

    initialiseConvexPolygon(points);
}

void LinearMinAreaEnclosingTriangleFinder::initialiseConvexPolygon(const vector<Point2d> &points) {
    polygon.clear();

    convexHull(points, polygon, CONVEX_HULL_CLOCKWISE);
}

double LinearMinAreaEnclosingTriangleFinder::findMinEnclosingTriangle(const vector<Point2d> &polygon,
                                                            vector<Point2d> &minEnclosingTriangle) {
    double minEnclosingTriangleArea = numeric_limits<double>::max();

    initialiseAlgorithmVariables();

    findMinEnclosingTriangle(minEnclosingTriangle, minEnclosingTriangleArea);

    return minEnclosingTriangleArea;
}

double LinearMinAreaEnclosingTriangleFinder::returnMinEnclosingTriangle(const vector<Point2d> &polygon,
                                                              vector<Point2d> &minEnclosingTriangle) {
    int nrOfPolygonPoints = polygon.size();

    for (int i = 0; i < 3; i++) {
        minEnclosingTriangle.push_back(polygon[i % nrOfPolygonPoints]);
    }

    return Geometry2D::areaOfTriangle(minEnclosingTriangle[0], minEnclosingTriangle[1], minEnclosingTriangle[2]);
}

void LinearMinAreaEnclosingTriangleFinder::initialiseAlgorithmVariables() {
    nrOfPoints = polygon.size();

    a = 1;
    b = 2;
    c = 0;
}

void LinearMinAreaEnclosingTriangleFinder::findMinEnclosingTriangle(vector<Point2d> &minEnclosingTriangle,
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
        Point2d gammaOfA;

        if ((gamma(a, gammaOfA)) && (intersectsBelow(gammaOfA, b))) {
            advance(b);
        } else {
            advance(a);
        }
    }
}

void LinearMinAreaEnclosingTriangleFinder::searchForBTangency() {
    Point2d gammaOfB;

    while (((gamma(b, gammaOfB)) && (intersectsBelow(gammaOfB, b))) &&
           (Numeric::greaterOrEqual(height(b), height(predecessor(a))))) {
        advance(b);
    }
}

bool LinearMinAreaEnclosingTriangleFinder::isNotBTangency() {
    Point2d gammaOfB;

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
    Point2d sideBMiddlePoint;

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
    Point2d midpointSideA = Geometry2D::middlePoint(vertexB, vertexC);
    Point2d midpointSideB = Geometry2D::middlePoint(vertexA, vertexC);
    Point2d midpointSideC = Geometry2D::middlePoint(vertexA, vertexB);

    bool sideAValid = (validationFlag == VALIDATION_SIDE_A_TANGENT)
                        ? (Geometry2D::areEqualPoints(midpointSideA, polygon[predecessor(a)]))
                        : (Geometry2D::isPointOnLineSegment(midpointSideA, sideAStartVertex, sideAEndVertex));

    bool sideBValid = (validationFlag == VALIDATION_SIDE_B_TANGENT)
                          ? (Geometry2D::areEqualPoints(midpointSideB, polygon[b]))
                          : (Geometry2D::isPointOnLineSegment(midpointSideB, sideBStartVertex, sideBEndVertex));

    bool sideCValid = Geometry2D::isPointOnLineSegment(midpointSideC, sideCStartVertex, sideCEndVertex);

    return (sideAValid && sideBValid && sideCValid);
}

void LinearMinAreaEnclosingTriangleFinder::updateMinEnclosingTriangle(vector<Point2d> &minEnclosingTriangle, double &minEnclosingTriangleArea) {
    area = Geometry2D::areaOfTriangle(vertexA, vertexB, vertexC);

    if (area < minEnclosingTriangleArea) {
        minEnclosingTriangle.clear();

        minEnclosingTriangle.push_back(vertexA);
        minEnclosingTriangle.push_back(vertexB);
        minEnclosingTriangle.push_back(vertexC);

        minEnclosingTriangleArea = area;
    }
}

bool LinearMinAreaEnclosingTriangleFinder::middlePointOfSideB(Point2d &middlePoint) {
    Point2d vertexA, vertexC;

    if ((!Geometry2D::lineIntersection(sideBStartVertex, sideBEndVertex, sideCStartVertex, sideCEndVertex, vertexA)) ||
        (!Geometry2D::lineIntersection(sideBStartVertex, sideBEndVertex, sideAStartVertex, sideAEndVertex, vertexC))) {
        return false;
    }

    middlePoint = Geometry2D::middlePoint(vertexA, vertexC);

    return true;
}

bool LinearMinAreaEnclosingTriangleFinder::intersectsBelow(const Point2d &gammaPoint, unsigned int polygonPointIndex) {
    double angleOfGammaAndPoint = Geometry2D::angleOfLineWrtOxAxis(polygon[polygonPointIndex], gammaPoint);

    return (intersects(angleOfGammaAndPoint, polygonPointIndex) == INTERSECTS_BELOW);
}

bool LinearMinAreaEnclosingTriangleFinder::intersectsAbove(const Point2d &gammaPoint, unsigned int polygonPointIndex) {
    double angleOfGammaAndPoint = Geometry2D::angleOfLineWrtOxAxis(gammaPoint, polygon[polygonPointIndex]);

    return (intersects(angleOfGammaAndPoint, polygonPointIndex) == INTERSECTS_ABOVE);
}

unsigned int LinearMinAreaEnclosingTriangleFinder::intersects(double angleOfGammaAndPoint, unsigned int polygonPointIndex) {
    double angleOfPointAndPredecessor = Geometry2D::angleOfLineWrtOxAxis(polygon[predecessor(polygonPointIndex)], polygon[polygonPointIndex]);
    double angleOfPointAndSuccessor = Geometry2D::angleOfLineWrtOxAxis(polygon[successor(polygonPointIndex)], polygon[polygonPointIndex]);
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

unsigned int LinearMinAreaEnclosingTriangleFinder::intersectsAboveOrBelow(unsigned int successorOrPredecessorIndex, unsigned int pointIndex) {
    if (height(successorOrPredecessorIndex) > height(pointIndex)) {
        return INTERSECTS_ABOVE;
    } else {
        return INTERSECTS_BELOW;
    }
}

bool LinearMinAreaEnclosingTriangleFinder::isFlushAngleBetweenPredecessorAndSuccessor(double &angleFlushEdge, double anglePredecessor,
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
    Point2d pointC = polygon[c];
    Point2d pointCPredecessor = polygon[predecessor(c)];

    Point2d polygonPoint = polygon[polygonPointIndex];

    return Geometry2D::distanceFromPointToLine(polygonPoint, pointC, pointCPredecessor);
}

double LinearMinAreaEnclosingTriangleFinder::height(const Point2d &polygonPoint) {
    Point2d pointC = polygon[c];
    Point2d pointCPredecessor = polygon[predecessor(c)];

    return Geometry2D::distanceFromPointToLine(polygonPoint, pointC, pointCPredecessor);
}

bool LinearMinAreaEnclosingTriangleFinder::gamma(unsigned int polygonPointIndex, Point2d &gammaPoint) {
    Point2d intersectionPoint1, intersectionPoint2;

    // Get intersection points if they exist
    if (!findGammaIntersectionPoints(polygonPointIndex, polygon[a], polygon[predecessor(a)], polygon[c],
                                     polygon[predecessor(c)], intersectionPoint1, intersectionPoint2)) {
        return false;
    }

    // Select the point which is on the same side of line C as the polygon
    if (Geometry2D::areOnTheSameSideOfLine(intersectionPoint1, polygon[successor(c)], polygon[c], polygon[predecessor(c)])) {
        gammaPoint = intersectionPoint1;
    } else {
        gammaPoint = intersectionPoint2;
    }

    return true;
}

Point2d LinearMinAreaEnclosingTriangleFinder::findVertexCOnSideB() {
    Point2d intersectionPoint1, intersectionPoint2;

    // Get intersection points if they exist
    if (!findGammaIntersectionPoints(predecessor(a), sideBStartVertex, sideBEndVertex, sideCStartVertex, sideCEndVertex,
                                     intersectionPoint1, intersectionPoint2)) {
        MAT_throw(UnexpectedBehaviourException, ERR_VERTEX_C_ON_SIDE_B);
    }

    // Select the point which is on the same side of line C as the polygon
    if (Geometry2D::areOnTheSameSideOfLine(intersectionPoint1, polygon[successor(c)], polygon[c], polygon[predecessor(c)])) {
        return intersectionPoint1;
    } else {
        return intersectionPoint2;
    }
}

bool LinearMinAreaEnclosingTriangleFinder::findGammaIntersectionPoints(unsigned int polygonPointIndex, const Point2d &side1StartVertex,
                                                             const Point2d &side1EndVertex, const Point2d &side2StartVertex,
                                                             const Point2d &side2EndVertex, Point2d &intersectionPoint1,
                                                             Point2d &intersectionPoint2) {
    vector<double> side1Params = lineEquationParameters(side1StartVertex, side1EndVertex);
    vector<double> side2Params = lineEquationParameters(side2StartVertex, side2EndVertex);

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

bool LinearMinAreaEnclosingTriangleFinder::areIdenticalLines(const vector<double> &side1Params, const vector<double> &side2Params,
                                                   double sideCExtraParam) {
    return (
        (Geometry2D::areIdenticalLines(side1Params[0], side1Params[1], -(side1Params[2]),
                                       side2Params[0], side2Params[1], -(side2Params[2]) - sideCExtraParam)) ||
        (Geometry2D::areIdenticalLines(side1Params[0], side1Params[1], -(side1Params[2]),
                                       side2Params[0], side2Params[1], -(side2Params[2]) + sideCExtraParam))
    );
}

bool LinearMinAreaEnclosingTriangleFinder::areIntersectingLines(const vector<double> &side1Params, const vector<double> &side2Params, double sideCExtraParam,
                                                      Point2d &intersectionPoint1, Point2d &intersectionPoint2) {
    return (
        (Geometry2D::lineIntersection(side1Params[0], side1Params[1], -(side1Params[2]),
                                      side2Params[0], side2Params[1], -(side2Params[2]) - sideCExtraParam,
                                      intersectionPoint1)) &&
        (Geometry2D::lineIntersection(side1Params[0], side1Params[1], -(side1Params[2]),
                                      side2Params[0], side2Params[1], -(side2Params[2]) + sideCExtraParam,
                                      intersectionPoint2))
    );
}

vector<double> LinearMinAreaEnclosingTriangleFinder::lineEquationParameters(const Point2d &p, const Point2d &q) {
    vector<double> lineEquationParameters;
    double a, b, c;

    Geometry2D::lineEquationDeterminedByPoints(p, q, a, b, c);

    lineEquationParameters.push_back(a);
    lineEquationParameters.push_back(b);
    lineEquationParameters.push_back(c);

    return lineEquationParameters;
}

void LinearMinAreaEnclosingTriangleFinder::advance(unsigned int &index) {
    index = successor(index);
}

unsigned int LinearMinAreaEnclosingTriangleFinder::successor(unsigned int index) {
    return ((index + 1) % nrOfPoints);
}

unsigned int LinearMinAreaEnclosingTriangleFinder::predecessor(unsigned int index) {
    return (index == 0) ? (nrOfPoints - 1)
                        : (index - 1);
}


// Constants
const bool LinearMinAreaEnclosingTriangleFinder::CONVEX_HULL_CLOCKWISE = true;

const unsigned int LinearMinAreaEnclosingTriangleFinder::INTERSECTS_BELOW      = 1;
const unsigned int LinearMinAreaEnclosingTriangleFinder::INTERSECTS_ABOVE      = 2;
const unsigned int LinearMinAreaEnclosingTriangleFinder::INTERSECTS_CRITICAL   = 3;
const unsigned int LinearMinAreaEnclosingTriangleFinder::INTERSECTS_LIMIT      = 4;

const string LinearMinAreaEnclosingTriangleFinder::ERR_NR_POINTS           = "The number of 2D points in the input vector should be greater than 0.";
const string LinearMinAreaEnclosingTriangleFinder::ERR_MIDPOINT_SIDE_B     = "The position of the middle point of side B could not be determined.";
const string LinearMinAreaEnclosingTriangleFinder::ERR_SIDE_B_GAMMA        = "The position of side B could not be determined, because gamma(b) could not be computed.";
const string LinearMinAreaEnclosingTriangleFinder::ERR_VERTEX_C_ON_SIDE_B  = "The position of the vertex C on side B could not be determined, because the considered lines do not intersect.";
const string LinearMinAreaEnclosingTriangleFinder::ERR_TRIANGLE_VERTICES   = "The position of the triangle vertices could not be determined, because the sides of the triangle do not intersect.";

const unsigned int LinearMinAreaEnclosingTriangleFinder::VALIDATION_SIDE_A_TANGENT   = 0;
const unsigned int LinearMinAreaEnclosingTriangleFinder::VALIDATION_SIDE_B_TANGENT   = 1;
const unsigned int LinearMinAreaEnclosingTriangleFinder::VALIDATION_SIDES_FLUSH      = 2;
