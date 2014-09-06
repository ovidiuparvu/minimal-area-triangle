#ifndef MINIMALAREATRIANGLETEST_HPP
#define MINIMALAREATRIANGLETEST_HPP

#include "gtest/gtest.h"


namespace triangletest {

    class MinimalAreaTriangleTest : public ::testing::Test {

        protected:

            bool validationFlag;    /*!< Flag indicating if the test results are valid */

        public:

            virtual ~MinimalAreaTriangleTest() {};

        protected:

            virtual void SetUp() {}
            virtual void TearDown() {}

            //! Run the test
            virtual void RunTest() = 0;

            //! Validate the results of the test
            virtual void ValidateTestResults() = 0;

    };

};


#endif
