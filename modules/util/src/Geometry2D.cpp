#include "triangle/util/Geometry2D.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

using namespace triangle::util;


double Geometry2D::angleOfLineWrtOxAxis(const cv::Point2f &a, const cv::Point2f &b) {
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
    if (fabs(angle2 - angle3) > 180) {
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

bool Geometry2D::slopeOfLine(const cv::Point2f &a, const cv::Point2f &b, double &slope) {
    double nominator = b.y - a.y;
    double denominator = b.x - a.x;

    if (denominator == 0) {
        return false;
    } else {
        slope = nominator / denominator;

        return true;
    }
}

double Geometry2D::distanceBtwPoints(const cv::Point2f &a, const cv::Point2f &b) {
    double xDiff = a.x - b.x;
    double yDiff = a.y - b.y;

    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

double Geometry2D::distanceBtwPoints(double x1, double y1, double x2, double y2) {
    double xDiff = x1 - x2;
    double yDiff = y1 - y2;

    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

double Geometry2D::distanceFromPointToLine(const cv::Point2f &a, const cv::Point2f &linePointB, const cv::Point2f &linePointC) {
    double term1 = linePointC.x - linePointB.x;
    double term2 = linePointB.y - a.y;
    double term3 = linePointB.x - a.x;
    double term4 = linePointC.y - linePointB.y;

    double nominator = fabs((term1 * term2) - (term3 * term4));
    double denominator = sqrt((term1 * term1) + (term4 * term4));

    return (nominator / denominator);
}

cv::Point2f Geometry2D::middlePoint(const cv::Point2f &a, const cv::Point2f &b) {
    double middleX = (a.x + b.x) / 2;
    double middleY = (a.y + b.y) / 2;

    return cv::Point2f(middleX, middleY);
}

bool Geometry2D::areOnTheSameSideOfLine(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &a, const cv::Point2f &b) {
    double a1, b1, c1;

    lineEquationDeterminedByPoints(a, b, a1, b1, c1);

    double p1OnLine = (a1 * p1.x) + (b1 * p1.y) + c1;
    double p2OnLine = (a1 * p2.x) + (b1 * p2.y) + c1;

    return (Numeric::sign(p1OnLine) == Numeric::sign(p2OnLine));
}


void Geometry2D::lineEquationDeterminedByPoints(const cv::Point2f &p, const cv::Point2f &q, double &a, double &b, double &c) {
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

bool Geometry2D:: areIdenticalLines(const cv::Point2f &a1, const cv::Point2f &b1, const cv::Point2f &a2, const cv::Point2f &b2) {
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

bool Geometry2D::lineIntersection(const cv::Point2f &a1, const cv::Point2f &b1, const cv::Point2f &a2, const cv::Point2f &b2, cv::Point2f &intersection) {
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

bool Geometry2D::lineIntersection(double a1, double b1, double c1, double a2, double b2, double c2, cv::Point2f &intersection) {
    double det = (a1 * b2) - (a2 * b1);

    if (!(Numeric::almostEqual(det, 0))) {
        intersection.x = ((c1 * b2) - (c2 * b1)) / (det);
        intersection.y = ((c2 * a1) - (c1 * a2)) / (det);

        return true;
    }

    return false;
}

double Geometry2D::angleBtwPoints(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &c) {
    cv::Point2f ab(b.x - a.x, b.y - a.y);
    cv::Point2f cb(b.x - c.x, b.y - c.y);

    double dotProduct   = (ab.x * cb.x + ab.y * cb.y);
    double crossProduct = (ab.x * cb.y - ab.y * cb.x);

    double alpha = atan2(crossProduct, dotProduct);

    return fabs(((alpha * 180) / PI));
}

double Geometry2D::areaOfTriangle(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &c) {
    double posTerm = (a.x * b.y) + (a.y * c.x) + (b.x * c.y);
    double negTerm = (b.y * c.x) + (a.x * c.y) + (a.y * b.x);

    double determinant = posTerm - negTerm;

    return fabs(determinant) / 2;
}

bool Geometry2D::isPointOnLineSegment(const cv::Point2f &point, const cv::Point2f &lineSegmentStart,
                                      const cv::Point2f &lineSegmentEnd) {
    double d1 = distanceBtwPoints(point, lineSegmentStart);
    double d2 = distanceBtwPoints(point, lineSegmentEnd);
    double lineSegmentLength = distanceBtwPoints(lineSegmentStart, lineSegmentEnd);

    return (Numeric::almostEqual(d1 + d2, lineSegmentLength));
}

bool Geometry2D::areEqualPoints(const cv::Point2f &point1, const cv::Point2f &point2) {
    return (Numeric::almostEqual(point1.x, point2.x) && Numeric::almostEqual(point1.y, point2.y));
}

bool Geometry2D::areCollinear(const cv::Point2f &point1, const cv::Point2f &point2, const cv::Point2f &point3) {
    double determinant = (point1.x * point2.y) + (point3.x * point1.y) + (point2.x * point3.y) -
                         (point3.x * point2.y) - (point1.x * point3.y) - (point2.x * point1.y);

    return (Numeric::almostEqual(determinant, 0));
}

bool Geometry2D::isPointOnEdge(const cv::Point2f &p, int nrOfRows, int nrOfCols) {
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

void Geometry2D::translate(cv::Point2f &point, const cv::Point2f &translation) {
    point.x += translation.x;
    point.y += translation.y;
}

void Geometry2D::inverseTranslate(cv::Point2f &point, const cv::Point2f &translation) {
    point.x -= translation.x;
    point.y -= translation.y;
}


// Constants
const double Geometry2D::PI = 3.14159265358979323846264338327950288419716939937510;
const int Geometry2D::MATRIX_START_INDEX = 1;
