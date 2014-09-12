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

//        runMinEnclosingTriangleFinder(points);
        runMinEnclosingTriangleFinder(std::vector<cv::Point2f>({cv::Point2f(1992.5, 190.6), cv::Point2f(2054.8, 192.4), cv::Point2f(2116.9, 197.7), cv::Point2f(2178.5, 206.3), cv::Point2f(2240.0, 215.4), cv::Point2f(2301.3, 225.5), cv::Point2f(2362.6, 235.7), cv::Point2f(2423.6, 247.1), cv::Point2f(2484.2, 260.5), cv::Point2f(2544.7, 274.0), cv::Point2f(2605.2, 287.7), cv::Point2f(2665.6, 302.0), cv::Point2f(2725.8, 316.9), cv::Point2f(2785.8, 332.4), cv::Point2f(2845.4, 349.1), cv::Point2f(2904.5, 367.3), cv::Point2f(2963.5, 386.0), cv::Point2f(3019.0, 412.7), cv::Point2f(3073.6, 441.0), cv::Point2f(3125.4, 473.7), cv::Point2f(3177.0, 507.0), cv::Point2f(3227.8, 541.2), cv::Point2f(3277.6, 576.7), cv::Point2f(3326.4, 613.5), cv::Point2f(3374.9, 650.8), cv::Point2f(3420.3, 691.4), cv::Point2f(3463.1, 734.6), cv::Point2f(3504.7, 778.9), cv::Point2f(3544.3, 824.8), cv::Point2f(3582.9, 871.6), cv::Point2f(3617.7, 921.0), cv::Point2f(3650.3, 971.8), cv::Point2f(3681.1, 1023.7), cv::Point2f(3709.1, 1076.9), cv::Point2f(3734.7, 1131.2), cv::Point2f(3760.4, 1185.5), cv::Point2f(3783.1, 1241.0), cv::Point2f(3803.7, 1297.2), cv::Point2f(3823.8, 1353.6), cv::Point2f(3842.9, 1410.3), cv::Point2f(3859.0, 1467.8), cv::Point2f(3870.2, 1526.3), cv::Point2f(3879.6, 1585.1), cv::Point2f(3888.5, 1643.9), cv::Point2f(3893.3, 1703.1), cv::Point2f(3895.6, 1762.3), cv::Point2f(3894.6, 1821.5), cv::Point2f(3892.4, 1880.7), cv::Point2f(3888.3, 1939.6), cv::Point2f(3882.6, 1998.5), cv::Point2f(3876.8, 2057.2), cv::Point2f(3869.3, 2115.8), cv::Point2f(3861.5, 2174.3), cv::Point2f(3853.7, 2232.8), cv::Point2f(3843.5, 2290.9), cv::Point2f(3831.6, 2348.6), cv::Point2f(3819.3, 2406.3), cv::Point2f(3807.0, 2463.9), cv::Point2f(3790.9, 2520.5), cv::Point2f(3774.4, 2577.0), cv::Point2f(3753.7, 2632.0), cv::Point2f(3732.1, 2686.6), cv::Point2f(3708.8, 2740.5), cv::Point2f(3684.4, 2793.9), cv::Point2f(3657.2, 2845.8), cv::Point2f(3625.2, 2895.0), cv::Point2f(3591.4, 2942.8), cv::Point2f(3557.3, 2990.4), cv::Point2f(3518.8, 3034.5), cv::Point2f(3479.5, 3077.9), cv::Point2f(3439.4, 3120.5), cv::Point2f(3398.7, 3162.6), cv::Point2f(3357.4, 3204.1), cv::Point2f(3315.1, 3244.5), cv::Point2f(3271.8, 3283.9), cv::Point2f(3228.2, 3322.9), cv::Point2f(3184.3, 3361.7), cv::Point2f(3140.3, 3400.3), cv::Point2f(3095.9, 3438.4), cv::Point2f(3050.2, 3475.0), cv::Point2f(3004.0, 3511.0), cv::Point2f(2954.5, 3542.2), cv::Point2f(2903.6, 3571.2), cv::Point2f(2852.2, 3599.2), cv::Point2f(2798.1, 3622.1), cv::Point2f(2743.2, 3642.8), cv::Point2f(2688.3, 3663.4), cv::Point2f(2633.0, 3683.1), cv::Point2f(2576.1, 3697.9), cv::Point2f(2519.1, 3712.4), cv::Point2f(2461.7, 3725.2), cv::Point2f(2403.3, 3733.3), cv::Point2f(2344.7, 3739.7), cv::Point2f(2285.8, 3743.4), cv::Point2f(2226.9, 3746.5), cv::Point2f(2167.9, 3748.8), cv::Point2f(2108.9, 3750.9), cv::Point2f(2049.7, 3749.8), cv::Point2f(1990.6, 3748.5), cv::Point2f(1931.5, 3746.4), cv::Point2f(1872.4, 3743.9), cv::Point2f(1813.3, 3741.2), cv::Point2f(1754.2, 3737.5), cv::Point2f(1695.4, 3730.0), cv::Point2f(1636.5, 3722.3), cv::Point2f(1577.8, 3714.1), cv::Point2f(1519.1, 3705.7), cv::Point2f(1460.5, 3695.7), cv::Point2f(1402.3, 3683.7), cv::Point2f(1344.1, 3671.5), cv::Point2f(1286.1, 3658.3), cv::Point2f(1228.2, 3644.6), cv::Point2f(1170.9, 3628.5), cv::Point2f(1113.9, 3611.0), cv::Point2f(1057.8, 3590.6), cv::Point2f(1003.1, 3566.2), cv::Point2f(949.4, 3539.7), cv::Point2f(896.1, 3512.1), cv::Point2f(844.3, 3481.8), cv::Point2f(793.4, 3449.6), cv::Point2f(745.4, 3413.1), cv::Point2f(697.6, 3376.2), cv::Point2f(651.5, 3337.1), cv::Point2f(607.3, 3295.7), cv::Point2f(564.6, 3252.6), cv::Point2f(522.0, 3209.4), cv::Point2f(480.9, 3164.7), cv::Point2f(441.9, 3118.0), cv::Point2f(405.4, 3069.2), cv::Point2f(370.1, 3019.4), cv::Point2f(335.4, 2969.1), cv::Point2f(301.7, 2918.2), cv::Point2f(270.2, 2865.7), cv::Point2f(238.7, 2813.3), cv::Point2f(207.9, 2760.3), cv::Point2f(178.9, 2706.3), cv::Point2f(151.1, 2651.6), cv::Point2f(126.1, 2595.5), cv::Point2f(103.8, 2538.1), cv::Point2f(84.4, 2479.6), cv::Point2f(66.3, 2420.5), cv::Point2f(50.9, 2360.7), cv::Point2f(37.2, 2300.4), cv::Point2f(25.2, 2239.6), cv::Point2f(16.7, 2178.2), cv::Point2f(9.5, 2116.5), cv::Point2f(3.5, 2054.7), cv::Point2f(0.0, 1992.6), cv::Point2f(1.6, 1930.3), cv::Point2f(8.6, 1868.2), cv::Point2f(16.4, 1806.2), cv::Point2f(27.8, 1744.7), cv::Point2f(43.8, 1684.1), cv::Point2f(63.0, 1624.4), cv::Point2f(84.0, 1565.3), cv::Point2f(105.2, 1506.2), cv::Point2f(132.0, 1449.3), cv::Point2f(160.3, 1393.2), cv::Point2f(189.9, 1337.7), cv::Point2f(221.6, 1283.3), cv::Point2f(253.8, 1229.3), cv::Point2f(287.8, 1176.3), cv::Point2f(324.1, 1124.8), cv::Point2f(360.7, 1073.6), cv::Point2f(397.8, 1022.7), cv::Point2f(436.0, 972.6), cv::Point2f(476.6, 924.4), cv::Point2f(517.9, 876.8), cv::Point2f(559.5, 829.5), cv::Point2f(601.2, 782.3), cv::Point2f(643.1, 735.2), cv::Point2f(685.8, 688.8), cv::Point2f(731.5, 645.4), cv::Point2f(779.8, 604.9), cv::Point2f(828.3, 564.7), cv::Point2f(876.9, 524.5), cv::Point2f(926.2, 485.3), cv::Point2f(977.3, 448.4), cv::Point2f(1029.6, 413.3), cv::Point2f(1083.2, 380.2), cv::Point2f(1138.4, 349.9), cv::Point2f(1197.0, 327.2), cv::Point2f(1255.7, 304.5), cv::Point2f(1314.6, 282.7), cv::Point2f(1374.5, 263.8), cv::Point2f(1434.5, 245.1), cv::Point2f(1494.7, 227.3), cv::Point2f(1556.5, 217.0), cv::Point2f(1618.5, 208.2), cv::Point2f(1680.6, 200.6), cv::Point2f(1742.8, 194.1), cv::Point2f(1805.3, 192.0), cv::Point2f(1867.7, 190.2), cv::Point2f(1930.1, 189.2)}));

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
