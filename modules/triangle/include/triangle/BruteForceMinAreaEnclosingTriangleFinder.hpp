#ifndef BRUTEFORCEMINAREAENCLOSINGTRIANGLEFINDER_HPP
#define BRUTEFORCEMINAREAENCLOSINGTRIANGLEFINDER_HPP

#include "triangle/MinAreaEnclosingTriangleFinder.hpp"

using namespace triangle;


namespace triangle {

    //! Class for computing the minimum area enclosing triangle for a given polygon
    /*!
     * This implementation has a cubic complexity (theta(n^3)) with respect to the number of points
     * defining the convex polygon.
     */
    class BruteForceMinAreaEnclosingTriangleFinder : public MinAreaEnclosingTriangleFinder {

        private:

            unsigned int a;     /*!< Index pointing to the first polygon point */
            unsigned int b;     /*!< Index pointing to the first polygon point */
            unsigned int c;     /*!< Index pointing to the third polygon point */

        public:

            BruteForceMinAreaEnclosingTriangleFinder();
            ~BruteForceMinAreaEnclosingTriangleFinder();

        private:

            //! Initialisation of the algorithm variables
            void initialiseAlgorithmVariables() override;

            //! Find the minimum area enclosing triangle for the given polygon
            /*!
             * \param minEnclosingTriangle      Minimum area triangle enclosing the given polygon
             * \param minEnclosingTriangleArea  Area of the minimum area enclosing triangle
             */
            void findMinEnclosingTriangle(std::vector<cv::Point2f> &minEnclosingTriangle,
                                          double &minEnclosingTriangleArea) override;

            //! Check if the lines defined by the provided indices are intersecting and valid
            /*!
             * \param firstLineFirstIndex   The index of the first polygon point defining the first line
             * \param firstLineSecondIndex  The index of the second polygon point defining the first line
             * \param secondLineFirstIndex  The index of the first polygon point defining the second line
             * \param secondLineSecondIndex The index of the second polygon point defining the second line
             */
            bool areValidIntersectingLines(unsigned int firstLineFirstIndex, unsigned int firstLineSecondIndex,
                                           unsigned int secondLineFirstIndex, unsigned int secondLineSecondIndex);

            //! Check if the given polygon points indices are valid
            /*! The polygon points are valid if they are pairwise disjoint
             *
             * \param firstPolygonPointIndex    The index of the first polygon point
             * \param secondPolygonPointIndex   The index of the second polygon point
             * \param thirdPolygonPointIndex    The index of the third polygon point
             */
            bool areValidPolygonPoints(unsigned int firstPolygonPointIndex, unsigned int secondPolygonPointIndex,
                                       unsigned int thirdPolygonPointIndex);

            //! Find the minimum area enclosing triangle considering that side B is tangent to the polygon
            /*! Side B is tangent to the polygon P in point polygon[b]
             *
             * \param minEnclosingTriangle      Minimum area triangle enclosing the given polygon
             * \param minEnclosingTriangleArea  Area of the minimum area enclosing triangle
             */
            void findMinEnclosingTriangleTangentSideB(std::vector<cv::Point2f> &minEnclosingTriangle,
                                                      double &minEnclosingTriangleArea);

            //! Find the minimum area enclosing triangle considering that side B is tangent to the polygon
            /*! Side B is tangent to the polygon P in point polygon[b]
             *
             * \param sideCParameters           Parameters (a1, b1, c1) defining side C
             * \param sideAParameters           Parameters (a2, b2, c2) defining side A
             * \param minEnclosingTriangle      Minimum area triangle enclosing the given polygon
             * \param minEnclosingTriangle      Minimum area triangle enclosing the given polygon
             * \param minEnclosingTriangleArea  Area of the minimum area enclosing triangle
             */
            void findMinEnclosingTriangleTangentSideB(const std::vector<double> &sideCParameters,
                                                      const std::vector<double> &sideAParameters,
                                                      std::vector<cv::Point2f> &minEnclosingTriangle,
                                                      double &minEnclosingTriangleArea);

            //! Find the minimum area enclosing triangle considering that side B is flush with a polygon edge
            /*! Side B is flush with the polygon edge defined by the points polygon[predecessor(b)] and polygon[b]
             *
             * \param minEnclosingTriangle      Minimum area triangle enclosing the given polygon
             * \param minEnclosingTriangleArea  Area of the minimum area enclosing triangle
             */
            void findMinEnclosingTriangleFlushSideB(std::vector<cv::Point2f> &minEnclosingTriangle,
                                                    double &minEnclosingTriangleArea);

            //! Check if the given lines are parallel
            /*! The lines are specified using the following format:
             *      ax + by + c = 0
             *
             * \param sideCParameters   The parameters for side/line C
             * \param sideAParameters   The parameters for side/line A
             */
            bool areParallelLines(const std::vector<double> &sideCParameters,
                                  const std::vector<double> &sideAParameters);

            //! Compute the coordinates of vertices C and A considering the given lines
            /*!
             * Prec: Side C and side A are intersecting
             *
             * \param sideCParameters   The parameters for side/line C
             * \param sideAParameters   The parameters for side/line A
             */
            void updateVerticesCAndA(const std::vector<double> &sideCParameters,
                                     const std::vector<double> &sideAParameters);

            //! Check if the given tangent side B is valid
            /*! Side B is valid if the angle described by it has a value which is between the values of the
             *  angles described by [b b-1] and [b+1 b]
             */
            bool isValidTangentSideB();

            //! Compute the enclosing triangle considering vertices A, B and C
            /*!
             * \param minEnclosingTriangle      Minimum area triangle enclosing the given polygon
             * \param minEnclosingTriangleArea  Area of the minimum area enclosing triangle
             */
            void computeEnclosingTriangle(std::vector<cv::Point2f> &minEnclosingTriangle,
                                          double &minEnclosingTriangleArea);

            //! Check if the found minimal triangle is valid
            /*!
             * This means that all midpoints of the triangle should touch the polygon
             */
            bool isValidMinimalTriangle() override;

        private:

            // Constants
            static const std::string ERR_PARALLEL_TRIANGLE_SIDES;

            static const double      POLYGON_POINT_TEST_THRESHOLD;

    };

};


#endif
