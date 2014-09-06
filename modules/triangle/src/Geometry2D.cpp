#include "triangle/Geometry2D.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

using namespace triangle;


double Geometry2D::angleOfLineWrtOxAxis(const Point2d &a, const Point2d &b) {
    double y = b.y - a.y;
    double x = b.x - a.x;

    double angle = (atan2(y, x) * 180 / PI);

    return (angle < 0) ? (angle + 360)
                       : angle;
}

bool Geometry2D::isAngleBetween(double angle1, double angle2, double angle3) {
    if ((((int)(angle2 - angle3)) % 180) > 0) {
        return ((angle3 < angle1) && (angle1 < angle2));
    } else {
        return ((angle2 < angle1) && (angle1 < angle3));
    }
}

bool Geometry2D::isOppositeAngleBetween(double angle1, double angle2, double angle3) {
    double angle1Opposite = oppositeAngle(angle1);

    return (isAngleBetween(angle1Opposite, angle2, angle3));
}

bool Geometry2D::isAngleBetweenNonReflex(double angle1, double angle2, double angle3) {
    if (abs(angle2 - angle3) > 180) {
        if (angle2 > angle3) {
            return ((angle2 < angle1) && (Numeric::lessOrEqual(angle1, 360))) || ((Numeric::lessOrEqual(0, angle1)) && (angle1 < angle3));
        } else {
            return ((angle3 < angle1) && (Numeric::lessOrEqual(angle1, 360))) || ((Numeric::lessOrEqual(0, angle1)) && (angle1 < angle2));
        }
    } else {
        return isAngleBetween(angle1, angle2, angle3);
    }
}

bool Geometry2D::isOppositeAngleBetweenNonReflex(double angle1, double angle2, double angle3) {
    double angle1Opposite = oppositeAngle(angle1);

    return (isAngleBetweenNonReflex(angle1Opposite, angle2, angle3));
}

double Geometry2D::oppositeAngle(double angle) {
    return (angle > 180) ? (angle - 180)
                         : (angle + 180);
}

bool Geometry2D::slopeOfLine(const Point2d &a, const Point2d &b, double &slope) {
    double nominator = b.y - a.y;
    double denominator = b.x - a.x;

    if (denominator == 0) {
        return false;
    } else {
        slope = nominator / denominator;

        return true;
    }
}

double Geometry2D::distanceBtwPoints(const Point2d &a, const Point2d &b) {
    double xDiff = a.x - b.x;
    double yDiff = a.y - b.y;

    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

double Geometry2D::distanceBtwPoints(double x1, double y1, double x2, double y2) {
    double xDiff = x1 - x2;
    double yDiff = y1 - y2;

    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

double Geometry2D::distanceFromPointToLine(const Point2d &a, const Point2d &linePointB, const Point2d &linePointC) {
    double term1 = linePointC.x - linePointB.x;
    double term2 = linePointB.y - a.y;
    double term3 = linePointB.x - a.x;
    double term4 = linePointC.y - linePointB.y;

    double nominator = abs((term1 * term2) - (term3 * term4));
    double denominator = sqrt((term1 * term1) + (term4 * term4));

    return (nominator / denominator);
}

Point2d Geometry2D::middlePoint(const Point2d &a, const Point2d &b) {
    double middleX = (a.x + b.x) / 2;
    double middleY = (a.y + b.y) / 2;

    return Point2d(middleX, middleY);
}

void Geometry2D::orthogonalLineToAnotherLineEdgePoints(const Point &a1, const Point &b1, Point &a2,
                                                       Point &b2, int nrOfRows, int nrOfCols) {
    if ((a1.x == b1.x) && (a1.y == b1.y)) {
        a2 = a1;
        b2 = b1;
    } else {
        if ((b1.x - a1.x) == 0) {   // Vertical line
            a2 = b1;
            b2 = b1;

            while (!isPointOnEdge(a2, nrOfRows, nrOfCols))
                a2.y--;

            while (!isPointOnEdge(b2, nrOfRows, nrOfCols))
                b2.y++;
        } else if (b1.y - a1.y == 0) {  // Horizontal line
            a2 = b1;
            b2 = b1;

            while (!isPointOnEdge(a2, nrOfRows, nrOfCols))
                a2.x--;

            while (!isPointOnEdge(b2, nrOfRows, nrOfCols))
                b2.x++;
        } else {                        // Otherwise
            double oldSlope = ((double)(b1.y - a1.y)) / (b1.x - a1.x);

            double newSlope = (-1) / (oldSlope);
            double intercept = b1.y - (newSlope * b1.x);

            a2 = b1;
            b2 = b1;

            while (!isPointOnEdge(a2, nrOfRows, nrOfCols)) {
                a2.x = a2.x - 1;
                a2.y = a2.x * newSlope + intercept;
            }

            while (!isPointOnEdge(b2, nrOfRows, nrOfCols)) {
                b2.x = b2.x + 1;
                b2.y = b2.x * newSlope + intercept;
            }
        }
    }
}

bool Geometry2D::areOnTheSameSideOfLine(const Point2d &p1, const Point2d &p2, const Point2d &a, const Point2d &b) {
    double a1, b1, c1;

    lineEquationDeterminedByPoints(a, b, a1, b1, c1);

    double p1OnLine = (a1 * p1.x) + (b1 * p1.y) + c1;
    double p2OnLine = (a1 * p2.x) + (b1 * p2.y) + c1;

    return (Numeric::sign(p1OnLine) == Numeric::sign(p2OnLine));
}


void Geometry2D::lineEquationDeterminedByPoints(const Point2d &p, const Point2d &q, double &a, double &b, double &c) {
    assert(Geometry2D::areEqualPoints(p, q) == false);

    a = q.y - p.y;
    b = p.x - q.x;
    c = ((-p.y) * b) - (p.x * a);
}

bool Geometry2D::areIdenticalLines(double a1, double b1, double c1, double a2, double b2, double c2) {
    double a1B2 = a1 * b2;
    double a2B1 = a2 * b1;
    double a1C2 = a1 * c2;
    double a2C1 = a2 * c1;
    double b1C2 = b1 * c2;
    double b2C1 = b2 * c1;

    return ((Numeric::almostEqual(a1B2, a2B1)) && (Numeric::almostEqual(b1C2, b2C1)) && (Numeric::almostEqual(a1C2, a2C1)));
}

bool Geometry2D:: areIdenticalLines(const Point2d &a1, const Point2d &b1, const Point2d &a2, const Point2d &b2) {
    double A1 = b1.y - a1.y;
    double B1 = a1.x - b1.x;
    double C1 = (a1.x * A1) + (a1.y * B1);

    double A2 = b2.y - a2.y;
    double B2 = a2.x - b2.x;
    double C2 = (a2.x * A2) + (a2.y * B2);

    double a1B2 = A1 * B2;
    double a2B1 = A2 * B1;
    double a1C2 = A1 * C2;
    double a2C1 = A2 * C1;
    double b1C2 = B1 * C2;
    double b2C1 = B2 * C1;

    return ((Numeric::almostEqual(a1B2, a2B1)) && (Numeric::almostEqual(b1C2, b2C1)) && (Numeric::almostEqual(a1C2, a2C1)));
}

bool Geometry2D::lineIntersection(const Point &a1, const Point &b1, const Point &a2, const Point &b2, Point &intersection) {
    double A1 = b1.y - a1.y;
    double B1 = a1.x - b1.x;
    double C1 = (a1.x * A1) + (a1.y * B1);

    double A2 = b2.y - a2.y;
    double B2 = a2.x - b2.x;
    double C2 = (a2.x * A2) + (a2.y * B2);

    double det = (A1 * B2) - (A2 * B1);

    if (!Numeric::almostEqual(det, 0)) {
        intersection.x = ((C1 * B2) - (C2 * B1)) / (det);
        intersection.y = ((C2 * A1) - (C1 * A2)) / (det);

        return true;
    }

    return false;
}

bool Geometry2D::lineIntersection(const Point2d &a1, const Point2d &b1, const Point2d &a2, const Point2d &b2, Point2d &intersection) {
    double A1 = b1.y - a1.y;
    double B1 = a1.x - b1.x;
    double C1 = (a1.x * A1) + (a1.y * B1);

    double A2 = b2.y - a2.y;
    double B2 = a2.x - b2.x;
    double C2 = (a2.x * A2) + (a2.y * B2);

    double det = (A1 * B2) - (A2 * B1);

    if (!Numeric::almostEqual(det, 0)) {
        intersection.x = ((C1 * B2) - (C2 * B1)) / (det);
        intersection.y = ((C2 * A1) - (C1 * A2)) / (det);

        return true;
    }

    return false;
}

bool Geometry2D::lineIntersection(double a1, double b1, double c1, double a2, double b2, double c2, Point2d &intersection) {
    double det = (a1 * b2) - (a2 * b1);

    if (!(Numeric::almostEqual(det, 0))) {
        intersection.x = ((c1 * b2) - (c2 * b1)) / (det);
        intersection.y = ((c2 * a1) - (c1 * a2)) / (det);

        return true;
    }

    return false;
}

bool Geometry2D::lineSegmentIntersection(const Point &a1, const Point &b1, const Point &a2, const Point &b2, Point &intersection) {
    if (lineIntersection(a1, b1, a2, b2, intersection)) {
        return (
                    isBetweenCoordinates<double, double>(intersection.x, a1.x, b1.x) &&
                    isBetweenCoordinates<double, double>(intersection.x, a2.x, b2.x) &&
                    isBetweenCoordinates<double, double>(intersection.y, a1.y, b1.y) &&
                    isBetweenCoordinates<double, double>(intersection.y, a2.y, b2.y)
               );
    }

    return false;
}

bool Geometry2D::lineCircleIntersection(Point2d a, Point2d b, const Point2d &circleOrigin,
                                        double radius, vector<Point2d> &intersectionPoints) {
    translate(a, Point2d(-circleOrigin.x, -circleOrigin.y));
    translate(b, Point2d(-circleOrigin.x, -circleOrigin.y));

    double A = b.y - a.y;
    double B = a.x - b.x;
    double C = (a.x * A) + (a.y * B);

    double A2 = A * A;
    double B2 = B * B;
    double C2 = C * C;
    double R2 = radius * radius;

    double delta = (4 * B2 * C2) - (4 * (A2 + B2) * (C2 - (R2 * A2)));

    if (delta > 0) {            /*!< Two intersection points */
        lineCircleTwoIntersectionPoints(circleOrigin, A, B, C, delta, intersectionPoints);

        return true;
    } else if (delta == 0) {    /*!< One intersection point */
        lineCircleOneIntersectionPoint(circleOrigin, A, B, C, delta, intersectionPoints);

        return true;
    }

    return false;
}

bool Geometry2D::lineSegmentCircleIntersection(const Point2d &a, const Point2d &b, const Point2d &circleOrigin,
                                               double radius, vector<Point2d> &intersectionPoints) {
    if (lineCircleIntersection(a, b, circleOrigin, radius, intersectionPoints)) {
        for (vector<Point2d>::iterator it = intersectionPoints.begin(); it != intersectionPoints.end(); ) {
            if (isBetweenCoordinates<float, double>((*it).x, a.x, b.x) &&
                isBetweenCoordinates<float, double>((*it).y, a.y, b.y)
               ) {
                ++it;
            } else {
                intersectionPoints.erase(it);
            }
        }

        return (intersectionPoints.size() > 0);
    }

    return false;
}

double Geometry2D::angleBtwPoints(const Point2d &a, const Point2d &b, const Point2d &c) {
    Point2d ab(b.x - a.x, b.y - a.y);
    Point2d cb(b.x - c.x, b.y - c.y);

    double dotProduct   = (ab.x * cb.x + ab.y * cb.y);
    double crossProduct = (ab.x * cb.y - ab.y * cb.x);

    double alpha = atan2(crossProduct, dotProduct);

    return abs(((alpha * 180) / PI));
}

vector<Point2d> Geometry2D::findPointsOnEdge(const vector<Point2d> &points,
                                             unsigned int nrOfRows,
                                             unsigned int nrOfCols) {
    vector<Point2d> pointsOnEdge;

    for (Point2d p : points) {
        if (isPointOnEdge(p, nrOfRows, nrOfCols)) {
            pointsOnEdge.push_back(p);
        }
    }

    return pointsOnEdge;
}

unsigned int Geometry2D::minimumDistancePointIndex(const vector<Point> &contour, const Point2d &origin) {
    double minDistance = numeric_limits<int>::max();
    double distance = 0.0;
    int nrOfPoints = contour.size();
    int minimumDistancePointIndex = -1;

    for (int i = 0; i < nrOfPoints; i++) {
        distance = distanceBtwPoints(contour[i], origin);

        if (distance < minDistance) {
            minDistance = distance;

            minimumDistancePointIndex = i;
        }
    }

    return minimumDistancePointIndex;
}

double Geometry2D::areaOfTriangle(const Point2d &a, const Point2d &b, const Point2d &c) {
    double posTerm = (a.x * b.y) + (a.y * c.x) + (b.x * c.y);
    double negTerm = (b.y * c.x) + (a.x * c.y) + (a.y * b.x);

    double determinant = posTerm - negTerm;

    return abs(determinant) / 2;
}

bool Geometry2D::isPointOnLineSegment(const Point2d &point, const Point2d &lineSegmentStart,
                                      const Point2d &lineSegmentEnd) {
    double d1 = distanceBtwPoints(point, lineSegmentStart);
    double d2 = distanceBtwPoints(point, lineSegmentEnd);
    double lineSegmentLength = distanceBtwPoints(lineSegmentStart, lineSegmentEnd);

    return (Numeric::almostEqual(d1 + d2, lineSegmentLength));
}

bool Geometry2D::areEqualPoints(const Point2d &point1, const Point2d &point2) {
    return (Numeric::almostEqual(point1.x, point2.x) && Numeric::almostEqual(point1.y, point2.y));
}

bool Geometry2D::areCollinear(const Point2d &point1, const Point2d &point2, const Point2d &point3) {
    double determinant = (point1.x * point2.y) + (point3.x * point1.y) + (point2.x * point3.y) -
                         (point3.x * point2.y) - (point1.x * point3.y) - (point2.x * point1.y);

    return (Numeric::almostEqual(determinant, 0));
}

bool Geometry2D::isPointOnEdge(const Point2d &p, int nrOfRows, int nrOfCols) {
    return (
              ((p.x <= MATRIX_START_INDEX) && (p.y > MATRIX_START_INDEX) && (p.y < nrOfCols)) ||
              ((p.x >= nrOfRows) && (p.y > MATRIX_START_INDEX) && (p.y < nrOfCols)) ||
              ((p.y <= MATRIX_START_INDEX) && (p.x > MATRIX_START_INDEX) && (p.x < nrOfRows)) ||
              ((p.y >= nrOfCols) && (p.x > MATRIX_START_INDEX) && (p.x < nrOfRows))
           );
}

template <typename T, typename U>
bool Geometry2D::isBetweenCoordinates(T c, U c1, U c2) {
    return ((std::min(c1, c2) <= c) && (c <= std::max(c1, c2)));
}

void Geometry2D::translate(Point2d &point, const Point2d &translation) {
    point.x += translation.x;
    point.y += translation.y;
}

void Geometry2D::inverseTranslate(Point2d &point, const Point2d &translation) {
    point.x -= translation.x;
    point.y -= translation.y;
}

void Geometry2D::lineCircleTwoIntersectionPoints(const Point2d &circleOrigin, double A, double B,
                                                 double C, double delta, vector<Point2d> &intersectionPoints) {
    double y1 = ((2 * B * C) + (sqrt(delta))) / (2 * ((A * A) + (B * B)));
    double y2 = ((2 * B * C) - (sqrt(delta))) / (2 * ((A * A) + (B * B)));

    double x1 = (C - (B * y1)) / (A);
    double x2 = (C - (B * y2)) / (A);

    Point2d firstIntersectionPoint(x1, y1);
    Point2d secondIntersectionPoint(x2, y2);

    inverseTranslate(firstIntersectionPoint, Point2d(-circleOrigin.x, -circleOrigin.y));
    inverseTranslate(secondIntersectionPoint, Point2d(-circleOrigin.x, -circleOrigin.y));

    intersectionPoints.push_back(firstIntersectionPoint);
    intersectionPoints.push_back(secondIntersectionPoint);
}

void Geometry2D::lineCircleOneIntersectionPoint(const Point2d &circleOrigin, double A, double B,
                                                double C, double delta, vector<Point2d> &intersectionPoints) {
    double y = (B * C) / ((A * A) + (B * B));
    double x = (C - (B * y)) / (A);

    Point2d intersectionPoint(x, y);

    inverseTranslate(intersectionPoint, Point2d(-circleOrigin.x, -circleOrigin.y));

    intersectionPoints.push_back(intersectionPoint);
}


// Constants
const double Geometry2D::PI = 3.14159265358979323846264338327950288419716939937510;
const int Geometry2D::MATRIX_START_INDEX = 1;
