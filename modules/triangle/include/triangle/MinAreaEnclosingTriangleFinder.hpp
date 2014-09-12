#ifndef MINAREAENCLOSINGTRIANGLEFINDER_HPP
#define MINAREAENCLOSINGTRIANGLEFINDER_HPP

#include "triangle/util/Geometry2D.hpp"

using namespace triangle;


namespace triangle {

    //! Class for computing the minimum area enclosing triangle for a given polygon
    class MinAreaEnclosingTriangleFinder {

        protected:

            cv::Point2f vertexA;                /*!< Vertex A of the current considered enclosing triangle */
            cv::Point2f vertexB;                /*!< Vertex B of the current considered enclosing triangle */
            cv::Point2f vertexC;                /*!< Vertex C of the current considered enclosing triangle */

            double area;                        /*!< Area of the current considered enclosing triangle */

            unsigned int nrOfPoints;            /*!< Number of points defining the polygon */

            std::vector<cv::Point2f> polygon;   /*!< Polygon for which the minimum area enclosing triangle
                                                     is computed */

        public:

            MinAreaEnclosingTriangleFinder();
            virtual ~MinAreaEnclosingTriangleFinder();

            //! Find the minimum area enclosing triangle for the given 2D point set
            /*!
             * Precondition: Number of points in the set is at least 1.
             *
             * \param points                        Set of points
             * \param minEnclosingTriangle          Minimum area triangle enclosing the given polygon
             */
            double find(const std::vector<cv::Point2f> &points,
                        std::vector<cv::Point2f> &minEnclosingTriangle);

        protected:

            //! Find the minimum area enclosing triangle for the given 2D point set
            /*!
             * \param points                        Set of points
             * \param minEnclosingTriangle          Minimum area triangle enclosing the given polygon
             */
            double findMinTriangle(const std::vector<cv::Point2f> &points,
                                   std::vector<cv::Point2f> &minEnclosingTriangle);

            //! Initialisation function for the class
            /*!
             * Initialise the polygon and other class' fields.
             *
             * \param points                Set of points
             * \param minEnclosingTriangle  Minimum area triangle enclosing the given polygon
             */
            void initialise(const std::vector<cv::Point2f> &points,
                            std::vector<cv::Point2f> &minEnclosingTriangle);

            //! Initialise polygon as the convex hull of the given set of points
            /*!
             * \param points Set of points
             */
            void initialiseConvexPolygon(const std::vector<cv::Point2f> &points);

            //! Find the minimum area enclosing triangle for the given polygon
            /*!
             * \param polygon                       Polygon of points for which the minimum area enclosing triangle will be  found
             * \param minEnclosingTriangle          Minimum area triangle enclosing the given polygon
             */
            double findMinEnclosingTriangle(const std::vector<cv::Point2f> &polygon,
                                            std::vector<cv::Point2f> &minEnclosingTriangle);

            //! Return the minimum area enclosing triangle in case the given polygon has at most three points
            /*!
             * \param polygon                       Polygon of points for which the minimum area enclosing triangle will be  found
             * \param minEnclosingTriangle          Minimum area triangle enclosing the given polygon
             */
            double returnMinEnclosingTriangle(const std::vector<cv::Point2f> &polygon,
                                              std::vector<cv::Point2f> &minEnclosingTriangle);

            //! Initialisation of the algorithm variables
            virtual void initialiseAlgorithmVariables() = 0;

            //! Find the minimum area enclosing triangle for the given polygon
            /*!
             * \param minEnclosingTriangle      Minimum area triangle enclosing the given polygon
             * \param minEnclosingTriangleArea  Area of the minimum area enclosing triangle
             */
            virtual void findMinEnclosingTriangle(std::vector<cv::Point2f> &minEnclosingTriangle,
                                                  double &minEnclosingTriangleArea) = 0;

            //! Check if the found minimal triangle is valid
            /*!
             * This means that all midpoints of the triangle should touch the polygon
             */
            virtual bool isValidMinimalTriangle() = 0;

            //! Update the current minimum area enclosing triangle if the newly obtained one has a smaller area
            /*!
             * \param minEnclosingTriangle      Minimum area triangle enclosing the given polygon
             * \param minEnclosingTriangleArea  Area of the minimum area triangle enclosing the given polygon
             */
            void updateMinEnclosingTriangle(std::vector<cv::Point2f> &minEnclosingTriangle,
                                            double &minEnclosingTriangleArea);

            //! Check if the given lines are identical or not
            /*!
             * The lines are specified as:
             *      ax + by + c = 0
             *  OR
             *      ax + by + c (+/-) sideCExtraParam = 0
             *
             * \param side1Params       Vector containing the values of a, b and c for side 1
             * \param side2Params       Vector containing the values of a, b and c for side 2
             * \param sideCExtraParam   Extra parameter for the flush edge C
             */
            bool areIdenticalLines(const std::vector<double> &side1Params, const std::vector<double> &side2Params,
                                   double sideCExtraParam);

            //! Check if the given lines intersect or not. If the lines intersect find their intersection points.
            /*!
             * The lines are specified as:
             *      ax + by + c = 0
             *  OR
             *      ax + by + c (+/-) sideCExtraParam = 0
             *
             * \param side1Params           Vector containing the values of a, b and c for side 1
             * \param side2Params           Vector containing the values of a, b and c for side 2
             * \param sideCExtraParam       Extra parameter for the flush edge C
             * \param intersectionPoint1    The first intersection point, if it exists
             * \param intersectionPoint2    The second intersection point, if it exists
             */
            bool areIntersectingLines(const std::vector<double> &side1Params, const std::vector<double> &side2Params,
                                      double sideCExtraParam, cv::Point2f &intersectionPoint1,
                                      cv::Point2f &intersectionPoint2);

            //! Get the line equation parameters "a", "b" and "c" for the line determined by points "p" and "q"
            /*!
             * The equation of the line is considered in the general form:
             * ax + by + c = 0
             *
             * \param p One point for defining the equation of the line
             * \param q Second point for defining the equation of the line
             */
            std::vector<double> lineEquationParameters(const cv::Point2f& p, const cv::Point2f &q);

            //! Advance the given index with one position
            /*!
             * \param index Index of the point
             */
            void advance(unsigned int &index);

            //! Return the succesor of the provided point index
            /*!
             * The succesor of the last polygon point is the first polygon point
             * (circular referencing)
             *
             * \param index Index of the point
             */
            unsigned int successor(unsigned int index);

            //! Return the predecessor of the provided point index
            /*!
             * The predecessor of the first polygon point is the last polygon point
             * (circular referencing)
             *
             * \param index Index of the point
             */
            unsigned int predecessor(unsigned int index);

        protected:

            // Constants
            static const bool CONVEX_HULL_CLOCKWISE;

            static const std::string ERR_NR_POINTS;
            static const std::string ERR_TRIANGLE_VERTICES;

    };

};


#endif
