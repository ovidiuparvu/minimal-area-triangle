#ifndef GEOMETRY2D_HPP
#define GEOMETRY2D_HPP

#include "triangle/util/Numeric.hpp"

#include <opencv2/imgproc/imgproc.hpp>


namespace triangle {

    namespace util {

        //! Two-dimensional geometric operations
        class Geometry2D {

            public:

                //! Get the angle of the line measured from the Ox axis in counterclockwise direction
                /*!
                 * The line is specified by points "a" and "b". The value of the angle is expressed in degrees.
                 *
                 * \param a cv::Point2f a
                 * \param b cv::Point2f b
                 */
                static double angleOfLineWrtOxAxis(const cv::Point2f &a, const cv::Point2f &b);

                //! Check if angle1 lies between angles 2 and 3
                /*!
                 * \param angle1 The angle which lies between angle2 and angle3 or not
                 * \param angle2 One of the boundary angles
                 * \param angle3 The other boundary angle
                 */
                static bool isAngleBetween(double angle1, double angle2, double angle3);

                //! Check if the opposite of angle1, ((angle1 + 180) % 360), lies between angles 2 and 3
                /*!
                 * \param angle1 The angle for which the opposite angle lies between angle2 and angle3 or not
                 * \param angle2 One of the boundary angles
                 * \param angle3 The other boundary angle
                 */
                static bool isOppositeAngleBetween(double angle1, double angle2, double angle3);

                //! Check if angle1 lies between non reflex angle determined by angles 2 and 3
                /*!
                 * \param angle1 The angle which lies between angle2 and angle3 or not
                 * \param angle2 One of the boundary angles
                 * \param angle3 The other boundary angle
                 */
                static bool isAngleBetweenNonReflex(double angle1, double angle2, double angle3);

                //! Check if the opposite of angle1, ((angle1 + 180) % 360), lies between non reflex angle determined by angles 2 and 3
                /*!
                 * \param angle1 The angle which lies between angle2 and angle3 or not
                 * \param angle2 One of the boundary angles
                 * \param angle3 The other boundary angle
                 */
                static bool isOppositeAngleBetweenNonReflex(double angle1, double angle2, double angle3);

                //! Return the angle opposite to the given angle
                /*!
                 * if (angle < 180) then
                 *      return (angle + 180);
                 * else
                 *      return (angle - 180);
                 * endif
                 *
                 * \param angle Angle
                 */
                static double oppositeAngle(double angle);

                //! Compute the slope of the line defined by points "a" and "b"
                /*!
                 * Returns true if the slope of the line can be computed and false otherwise.
                 *
                 * \param a cv::Point2f a
                 * \param b cv::Point2f b
                 * \param slope Slope of the line if it is different from (+/-)infinity
                 */
                static bool slopeOfLine(const cv::Point2f &a, const cv::Point2f &b, double &slope);

                //! Compute the distance between two points
                /*! Compute the Euclidean distance between two points
                 *
                 * \param a cv::Point2f a
                 * \param b cv::Point2f b
                 */
                static double distanceBtwPoints(const cv::Point2f &a, const cv::Point2f &b);

                //! Compute the distance between two points
                /*! Compute the Euclidean distance between two points
                 *
                 * \param x1 The x-coordinate of the first point
                 * \param y1 The y-coordinate of the first point
                 * \param x2 The x-coordinate of the second point
                 * \param y2 The y-coordinate of the second point
                 */
                static double distanceBtwPoints(double x1, double y1, double x2, double y2);

                //! Compute the distance from a point "a" to a line specified by two points "B" and "C"
                /*!
                 * Formula used:
                 *
                 * \f$ distance = \frac{\mid (x_c - x_b)(y_b - y_a) - (x_b - x_a)(y_c - y_b) \mid}{\sqrt{(x_c - x_b)^{2} + (y_c - y_b)^{2}}} \f$
                 *
                 * Reference: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
                 *
                 * \param a             cv::Point2f from which the distance is measures
                 * \param linePointB    One of the points determining the line
                 * \param linePointC    One of the points determining the line
                 */
                static double distanceFromPointToLine(const cv::Point2f &a, const cv::Point2f &linePointB, const cv::Point2f &linePointC);

                //! Get the point in the middle of the segment determined by points "a" and "b"
                /*!
                 * \param a cv::Point2f a
                 * \param b cv::Point2f b
                 */
                static cv::Point2f middlePoint(const cv::Point2f &a, const cv::Point2f &b);

                //! Check if p1 and p2 are on the same side of the line determined by points a and b
                /*!
                 * \param p1    Point p1
                 * \param p2    Point p2
                 * \param a     First point for determining line
                 * \param b     Second point for determining line
                 */
                static bool areOnTheSameSideOfLine(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &a, const cv::Point2f &b);

                //! Get the values of "a", "b" and "c" of the line equation ax + by + c = 0 knowing that point "p" and "q" are on the line
                /*!
                 * a = q.y - p.y
                 * b = p.x - q.x
                 * c = - (p.x * a) - (p.y * b)
                 *
                 * \param p cv::Point2f p
                 * \param q cv::Point2f q
                 * \param a Parameter "a" from the line equation
                 * \param b Parameter "b" from the line equation
                 * \param c Parameter "c" from the line equation
                 */
                static void lineEquationDeterminedByPoints(const cv::Point2f &p, const cv::Point2f &q, double &a, double &b, double &c);

                //! Check if two lines are identical
                /*!
                 * Lines are be specified in the following form:
                 *      A1x + B1x = C1
                 *      A2x + B2x = C2
                 *
                 * If (A1/A2) == (B1/B2) == (C1/C2), then the lines are identical
                 *                                   else they are not
                 *
                 * \param a1 A1
                 * \param b1 B1
                 * \param c1 C1
                 * \param a2 A2
                 * \param b2 B2
                 * \param c2 C2
                 */
                static bool areIdenticalLines(double a1, double b1, double c1, double a2, double b2, double c2);

                //! Check if two lines are identical
                /*!
                 * The lines are specified by a pair of points each. If they are identical, then
                 * the function returns true, else it returns false.
                 *
                 * Lines can be specified in the following form:
                 *      A1x + B1x = C1
                 *      A2x + B2x = C2
                 *
                 * If (A1/A2) == (B1/B2) == (C1/C2), then the lines are identical
                 *                                   else they are not
                 *
                 * \param a1 First point for determining the first line
                 * \param b1 Second point for determining the first line
                 * \param a2 First point for determining the second line
                 * \param b2 Second point for determining the second line
                 */
                static bool areIdenticalLines(const cv::Point2f &a1, const cv::Point2f &b1, const cv::Point2f &a2, const cv::Point2f &b2);

                //! Determine the intersection point of two lines, if this point exists
                /*! Two lines intersect if they are not parallel (Parallel lines intersect at
                 * +/- infinity, but we do not consider this case here).
                 *
                 * The lines are specified by a pair of points each. If they intersect, then
                 * the function returns true, else it returns false.
                 *
                 * Lines can be specified in the following form:
                 *      A1x + B1x = C1
                 *      A2x + B2x = C2
                 *
                 * If det (= A1xB2 - A2xB1) == 0, then lines are parallel
                 *                                else they intersect
                 *
                 * If they intersect, then let us denote the intersection point with P(x, y) where:
                 *      x = (C1xB2 - C2xB1) / (det)
                 *      y = (C2xA1 - C1xA2) / (det)
                 *
                 * \param a1 First point for determining the first line
                 * \param b1 Second point for determining the first line
                 * \param a2 First point for determining the second line
                 * \param b2 Second point for determining the second line
                 * \param intersection The intersection point, if this point exists
                 */
                static bool lineIntersection(const cv::Point2f &a1, const cv::Point2f &b1, const cv::Point2f &a2, const cv::Point2f &b2, cv::Point2f &intersection);

                //! Determine the intersection point of two lines, if this point exists
                /*! Two lines intersect if they are not parallel (Parallel lines intersect at
                 * +/- infinity, but we do not consider this case here).
                 *
                 * The lines are specified in the following form:
                 *      A1x + B1x = C1
                 *      A2x + B2x = C2
                 *
                 * If det (= A1xB2 - A2xB1) == 0, then lines are parallel
                 *                                else they intersect
                 *
                 * If they intersect, then let us denote the intersection point with P(x, y) where:
                 *      x = (C1xB2 - C2xB1) / (det)
                 *      y = (C2xA1 - C1xA2) / (det)
                 *
                 * \param a1 A1
                 * \param b1 B1
                 * \param c1 C1
                 * \param a2 A2
                 * \param b2 B2
                 * \param c2 C2
                 * \param intersection The intersection point, if this point exists
                 */
                static bool lineIntersection(double a1, double b1, double c1, double a2, double b2, double c2, cv::Point2f &intersection);

                //! Compute the angle between three points
                /*! Compute the angle between the lines determined by
                 * points A, B and B, C
                 *
                 * \param a cv::Point2f a
                 * \param b cv::Point2f b
                 * \param c cv::Point2f c
                 */
                static double angleBtwPoints(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &c);

                //! Compute the area of a triangle defined by three points
                /*!
                 * The area is computed using the determinant method.
                 * An example is presented at http://demonstrations.wolfram.com/TheAreaOfATriangleUsingADeterminant/
                 * (Last access: 10.07.2013)
                 *
                 * \param a cv::Point2f a
                 * \param b cv::Point2f b
                 * \param c cv::Point2f c
                 */
                static double areaOfTriangle(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &c);

                //! Check if one point lies between two other points
                /*!
                 * \param point             Point lying possibly outside the line segment
                 * \param lineSegmentStart  First point determining the line segment
                 * \param lineSegmentEnd    Second point determining the line segment
                 */
                static bool isPointOnLineSegment(const cv::Point2f &point, const cv::Point2f &lineSegmentStart,
                                                 const cv::Point2f &lineSegmentEnd);

                //! Check if points point1 and point2 are equal or not
                /*!
                 * \param point1 One point
                 * \param point2 The other point
                 */
                static bool areEqualPoints(const cv::Point2f &point1, const cv::Point2f &point2);

                //! Check if the three points are collinear
                /*!
                 * \param point1 Point 1
                 * \param point2 Point 2
                 * \param point3 Point 3
                 */
                static bool areCollinear(const cv::Point2f &point1, const cv::Point2f &point2, const cv::Point2f &point3);

            private:

                //! Check if the given point is on the edge
                /*!
                 *  A point "p" is considered to be on the edge if:
                 *      ((p.x == 1) && (p.y > 1) && (p.y < nrOfCols)) OR
                 *      ((p.x == nrOfRows) && (p.y > 1) && (p.y < nrOfCols)) OR
                 *      ((p.y == 1) && (p.x > 1) && (p.x < nrOfRows)) OR
                 *      ((p.y == nrOfCols) && (p.x > 1) && (p.x < nrOfRows))
                 *
                 *  \param p cv::Point2f p
                 *  \param nrOfRows The number of rows
                 *  \param nrOfCols The number of columns
                 */
                static bool isPointOnEdge(const cv::Point2f &p, int nrOfRows, int nrOfCols);

                //! Check if the coordinate c lies between c1 and c2
                /*!
                 * \param c Coordinate c
                 * \param c1 Coordinate c1
                 * \param c2 Coordinate c2
                 */
                template <typename T, typename U>
                static bool isBetweenCoordinates(T c, U c1, U c2);

                //! Translate a point by the given values
                /*!
                 * \param point The point
                 * \param translation Translation values
                 */
                static void translate(cv::Point2f &point, const cv::Point2f &translation);

                //! Inverse translate a point by the given values
                /*!
                 * \param point The point
                 * \param translation Translation values
                 */
                static void inverseTranslate(cv::Point2f &point, const cv::Point2f &translation);


            public:

                // Constants
                static const double PI;
                static const int MATRIX_START_INDEX;

        };

    };

};


#endif
