#include "triangle/exception/ExceptionHandler.hpp"
#include "triangle/Geometry2D.hpp"
#include "triangle/LinearMinAreaEnclosingTriangleFinder.hpp"

#include <ctime>
#include <iostream>

using namespace std;
using namespace cv;
using namespace triangle;

const string WIN_MIN_AREA_TRIANGLE  = "Minimum area enclosing triangle";

const int KEY_ESC = 27;

const int RADIUS                    = 1;
const int LINE_THICKNESS            = 50;
const int NR_RAND_POLYGONS          = 50;
const int MAX_POLYGON_POINTS        = 100;
const int POLYGON_POINT_X_MAX       = 500;
const int POLYGON_POINT_Y_MAX       = 500;

const double POINT_IN_TRIANGLE_THRESH = 1E-4;


// Generate a new random set of points
vector<Point2d> generateRandomSetOf2DPoints(int nrOfPoints) {
    vector<Point2d> points;

    for (int i = 0; i < nrOfPoints; i++) {
        points.push_back(Point2d((rand() % POLYGON_POINT_X_MAX) + POLYGON_POINT_X_MAX,
                                 (rand() % POLYGON_POINT_Y_MAX) + POLYGON_POINT_Y_MAX));
    }

    return points;
}

// Print the polygon points
void printPolygon(const vector<Point2d> &points) {
    vector<Point2d> polygon;

    convexHull(points, polygon);

    // Print the polygon points
    cout << "Polygon points: ";

    for (const Point2d &point : polygon) {
        cout << "(" << point.x << ", " << point.y << ") ";
    }

    cout << endl;
}

// Output the results for the minimum area enclosing triangle
void outputMinEnclosingTriangleFinderResults(const vector<Point2d> &minEnclosingTriangle, const vector<Point2d> &points) {
    Mat image = Mat::zeros(POLYGON_POINT_X_MAX * 3, POLYGON_POINT_Y_MAX * 3, CV_32FC3);
    Mat flippedImage = Mat::zeros(POLYGON_POINT_X_MAX * 3, POLYGON_POINT_Y_MAX * 3, CV_32FC3);

    // Draw minimum area enclosing triangle
    for (unsigned int i = 0; i < minEnclosingTriangle.size(); i++) {
        line(image, minEnclosingTriangle[i], minEnclosingTriangle[(i + 1) % minEnclosingTriangle.size()], Scalar(0, 255, 255), LINE_THICKNESS);
    }

    // Draw convex hull points
    for (const Point2d &point : points) {
        circle(image, point, RADIUS, Scalar(0, 0, 255), LINE_THICKNESS);
    }

    printPolygon(points);

    // Flip image wrt Ox axis and show it
    flip(image, flippedImage, 0);

    namedWindow(WIN_MIN_AREA_TRIANGLE, WINDOW_NORMAL);
    imshow(WIN_MIN_AREA_TRIANGLE, flippedImage);
}

// Check if all the points are enclosed by the minimal enclosing triangle
bool arePointsEnclosed(const vector<Point2d> &points, const vector<Point2d> &triangle) {
    double distance = 0;

    for (const Point2d &point : points) {
        distance = pointPolygonTest(triangle, point, true);

        if (distance < -(POINT_IN_TRIANGLE_THRESH)) {
            return false;
        }
    }

    return true;
}

// Check if all the triangle sides' middle points touch the convex hull of the given set of points
bool isTriangleTouchingPolygon(const vector<Point2d> &convexPolygon, const vector<Point2d> &triangle) {
    int nrOfPolygonPoints = convexPolygon.size();

    for (int i = 0; i < 3; i++) {
        bool isTouching = false;
        Point2d middlePoint = Geometry2D::middlePoint(triangle[i], triangle[(i + 1) % 3]);

        for (int j = 0; j < nrOfPolygonPoints; j++) {
            if (Geometry2D::isPointOnLineSegment(middlePoint, convexPolygon[j],
                                                 convexPolygon[(j + 1) % nrOfPolygonPoints])) {
                isTouching = true;
            }
        }

        if (!isTouching) {
            return false;
        }
    }

    return true;
}

// Check if at least one side of the triangle is flush with an edge of the polygon
bool isOneEdgeFlush(const vector<Point2d> &convexPolygon, const vector<Point2d> &triangle) {
    int nrOfPolygonPoints = convexPolygon.size();

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < nrOfPolygonPoints; j++) {
            if ((Geometry2D::isPointOnLineSegment(convexPolygon[j], triangle[i],
                                                  triangle[(i + 1) % 3])) &&
                (Geometry2D::isPointOnLineSegment(convexPolygon[(j + 1) % nrOfPolygonPoints], triangle[i],
                                                  triangle[(i + 1) % 3]))) {
                return true;
            }
        }
    }

    return false;
}

// Check if the minimum enclosing triangle encloses all points
bool isValidTriangle(const vector<Point2d> &points, const vector<Point2d> &triangle) {
    vector<Point2d> convexPolygon;

    convexHull(points, convexPolygon, true);

    return (
        (arePointsEnclosed(points, triangle)) &&
        (isTriangleTouchingPolygon(convexPolygon, triangle)) &&
        (isOneEdgeFlush(convexPolygon, triangle))
    );
}

// Run the minimum area enclosing triangle program
void runMinEnclosingTriangleFinder(const vector<Point2d> &points) {
    vector<Point2d> minEnclosingTriangle;
    double area = 0;

    // Find the minimum area enclosing triangle
    area = LinearMinAreaEnclosingTriangleFinder().find(points, minEnclosingTriangle);

    // Validate the found triangle
    assert(isValidTriangle(points, minEnclosingTriangle));

    cout << "The area of the minimum area enclosing triangle is: " << area << endl;

    outputMinEnclosingTriangleFinderResults(minEnclosingTriangle, points);
}

// Run the minimum area enclosing triangle program using randomly generated sets of points
void runMinEnclosingTriangleFinderUsingRandomPolygons() {
    char key = 0;

    // Initialise the seed - milliseconds is enough as this program is not to be run in parallel
    srand(time(0));

    while (key != KEY_ESC) {
        int nrOfPoints = rand() % MAX_POLYGON_POINTS;

        nrOfPoints = (nrOfPoints == 0) ? 1
                                       : nrOfPoints;

        vector<Point2d> points = generateRandomSetOf2DPoints(nrOfPoints);

        runMinEnclosingTriangleFinder(points);

        key = waitKey();
    }
}

// Run the minimum area enclosing triangle program
void runMinEnclosingTriangleFinder() {
    runMinEnclosingTriangleFinderUsingRandomPolygons();
}

// Main function
int main(int argc, char** argv) {
    try {
        runMinEnclosingTriangleFinder();
    } catch (const std::exception &ex) {
        ExceptionHandler::printErrorMessage(ex);
    }

    return EXEC_SUCCESS_CODE;
}
