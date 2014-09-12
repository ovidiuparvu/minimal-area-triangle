#include "triangle/BruteForceMinAreaEnclosingTriangleFinder.hpp"
#include "triangle/LinearMinAreaEnclosingTriangleFinder.hpp"
#include "triangle/exception/ExceptionHandler.hpp"

#include "opencv2/highgui/highgui.hpp"

#include <ctime>
#include <iostream>

using namespace triangle;
using namespace triangle::util;

const std::string WIN_MIN_AREA_TRIANGLE  = "Minimum area enclosing triangle";

const int KEY_ESC = 27;

const int RADIUS                    = 1;
const int LINE_THICKNESS            = 50;
const int NR_RAND_POLYGONS          = 50;
const int MAX_POLYGON_POINTS        = 100;
const int POLYGON_POINT_X_MAX       = 500;
const int POLYGON_POINT_Y_MAX       = 500;

const double POINT_IN_TRIANGLE_THRESH = 1E-4;


// Generate a new random set of points
std::vector<cv::Point2f> generateRandomSetOf2DPoints(int nrOfPoints) {
    std::vector<cv::Point2f> points;

    for (int i = 0; i < nrOfPoints; i++) {
        points.push_back(cv::Point2f((rand() % POLYGON_POINT_X_MAX) + POLYGON_POINT_X_MAX,
                                 (rand() % POLYGON_POINT_Y_MAX) + POLYGON_POINT_Y_MAX));
    }

    return points;
}

// Print the polygon points
void printPolygon(const std::vector<cv::Point2f> &points) {
    std::vector<cv::Point2f> polygon;

    convexHull(points, polygon);

    // Print the polygon points
    std::cout << "Polygon points: ";

    for (const cv::Point2f &point : polygon) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }

    std::cout << std::endl;
}

// Output the results for the minimum area enclosing triangle
void outputMinEnclosingTriangleFinderResults(const std::vector<cv::Point2f> &bruteForceMinEnclosingTriangle,
                                             const std::vector<cv::Point2f> &linearMinEnclosingTriangle,
                                             const std::vector<cv::Point2f> &points) {
    cv::Mat image = cv::Mat::zeros(POLYGON_POINT_X_MAX * 3, POLYGON_POINT_Y_MAX * 3, CV_32FC3);
    cv::Mat flippedImage = cv::Mat::zeros(POLYGON_POINT_X_MAX * 3, POLYGON_POINT_Y_MAX * 3, CV_32FC3);

    // Draw brute force minimum area enclosing triangle
    for (unsigned int i = 0; i < bruteForceMinEnclosingTriangle.size(); i++) {
        cv::line(image, bruteForceMinEnclosingTriangle[i],
                 bruteForceMinEnclosingTriangle[(i + 1) % bruteForceMinEnclosingTriangle.size()],
                 cv::Scalar(0, 255, 255), LINE_THICKNESS);
    }

    // Draw linear minimum area enclosing triangle
    for (unsigned int i = 0; i < linearMinEnclosingTriangle.size(); i++) {
        cv::line(image, linearMinEnclosingTriangle[i],
                 linearMinEnclosingTriangle[(i + 1) % linearMinEnclosingTriangle.size()],
                 cv::Scalar(255, 0, 255), LINE_THICKNESS);
    }

    // Draw convex hull points
    for (const cv::Point2f &point : points) {
        cv::circle(image, point, RADIUS, cv::Scalar(0, 0, 255), LINE_THICKNESS);
    }

    printPolygon(points);

    // Flip image wrt Ox axis and show it
    cv::flip(image, flippedImage, 0);

    cv::namedWindow(WIN_MIN_AREA_TRIANGLE, cv::WINDOW_NORMAL);
    cv::imshow(WIN_MIN_AREA_TRIANGLE, flippedImage);
}

// Check if all the points are enclosed by the minimal enclosing triangle
bool arePointsEnclosed(const std::vector<cv::Point2f> &points, const std::vector<cv::Point2f> &triangle) {
    double distance = 0;

    for (const cv::Point2f &point : points) {
        distance = pointPolygonTest(triangle, point, true);

        if (distance < -(POINT_IN_TRIANGLE_THRESH)) {
            return false;
        }
    }

    return true;
}

// Check if all the triangle sides' middle points touch the convex hull of the given set of points
bool isTriangleTouchingPolygon(const std::vector<cv::Point2f> &convexPolygon, const std::vector<cv::Point2f> &triangle) {
    int nrOfPolygonPoints = convexPolygon.size();

    for (int i = 0; i < 3; i++) {
        bool isTouching = false;
        cv::Point2f middlePoint = Geometry2D::middlePoint(triangle[i], triangle[(i + 1) % 3]);

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
bool isOneEdgeFlush(const std::vector<cv::Point2f> &convexPolygon, const std::vector<cv::Point2f> &triangle) {
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
bool isValidTriangle(const std::vector<cv::Point2f> &points, const std::vector<cv::Point2f> &triangle) {
    std::vector<cv::Point2f> convexPolygon;

    convexHull(points, convexPolygon, true);

    return (
        (arePointsEnclosed(points, triangle)) &&
        (isTriangleTouchingPolygon(convexPolygon, triangle)) &&
        (isOneEdgeFlush(convexPolygon, triangle))
    );
}

// Find the minimal area enclosing triangle for the given set of points using the specified "T" approach
template <typename T>
void findMinAreaEnclosingTriangle(const std::vector<cv::Point2f> &points,
                                  std::vector<cv::Point2f> &minAreaEnclosingTriangle) {
    double area = 0;

    // Find the minimum area enclosing triangle
    area = T().find(points, minAreaEnclosingTriangle);

    // Validate the found triangle
    assert(isValidTriangle(points, minAreaEnclosingTriangle));

    std::cout << "The area of the minimum area enclosing triangle is: " << area << std::endl;
}

// Run the minimum area enclosing triangle program
void runMinEnclosingTriangleFinder(const std::vector<cv::Point2f> &points) {
    std::vector<cv::Point2f> bruteForceMinEnclosingTriangle;
    std::vector<cv::Point2f> linearMinEnclosingTriangle;

    findMinAreaEnclosingTriangle<BruteForceMinAreaEnclosingTriangleFinder>(points, bruteForceMinEnclosingTriangle);
    findMinAreaEnclosingTriangle<LinearMinAreaEnclosingTriangleFinder>(points, linearMinEnclosingTriangle);

    outputMinEnclosingTriangleFinderResults(bruteForceMinEnclosingTriangle, linearMinEnclosingTriangle, points);
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

        std::vector<cv::Point2f> points = generateRandomSetOf2DPoints(nrOfPoints);

        runMinEnclosingTriangleFinder(points);

        key = cv::waitKey();
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
