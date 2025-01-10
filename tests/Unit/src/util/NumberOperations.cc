/******************************************************************************
 * @brief Unit test for NumberOperations utility class.
 *
 * @file NumberOperations.cc
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/util/NumberOperations.hpp"

/// \cond
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

/// \endcond

/******************************************************************************
 * @brief Test the functionality of the Clamp function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-16
 ******************************************************************************/
TEST(NumOpsTest, Clamp)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 6;
    const double aValues[nTestValuesLength]   = {1.0, 0.0, 567.0, 0.05, -1.0, -89.3};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -5.0, 0.05, -3.0, 50.0};
    const double aMaximums[nTestValuesLength] = {2.0, 1.0, 0.0, 0.04, 0.0, -100.0};
    const double aOutput[nTestValuesLength]   = {1.0, 0.0, 0.0, 0.05, -1.0, 50.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate clamp values.
        double dResult = numops::Clamp(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);    // output check.
    }
}

/******************************************************************************
 * @brief Test the functionality of the Bounded function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-16
 ******************************************************************************/
TEST(NumOpsTest, Bounded)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 6;
    const double aValues[nTestValuesLength]   = {1.0, 0.0, 567.0, 0.05, -1.0, -89.3};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -5.0, 0.05, -3.0, 50.0};
    const double aMaximums[nTestValuesLength] = {2.0, 1.0, 0.0, 0.04, 0.0, -100.0};
    const bool aOutput[nTestValuesLength]     = {true, true, false, false, true, false};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        bool bResult = numops::Bounded(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(bResult, aOutput[nIter]);    // output check.
    }
}

/******************************************************************************
 * @brief Test the functionality of the MapRange function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
TEST(NumOpsTest, MapRange)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                  = 4;
    const double aValues[nTestValuesLength]      = {0.0, -1.0, 2.0, 0};
    const double aOldMinimums[nTestValuesLength] = {-1.0, -1.0, -4.0, -180};
    const double aOldMaximums[nTestValuesLength] = {1.0, 1.0, 4.0, 180};
    const double aNewMinimums[nTestValuesLength] = {0.0, 0.0, -2.0, 0};
    const double aNewMaximums[nTestValuesLength] = {2.0, 2.0, 2.0, 360};
    const double aOutput[nTestValuesLength]      = {1.0, 0.0, 1.0, 180};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::MapRange(aValues[nIter], aOldMinimums[nIter], aOldMaximums[nIter], aNewMinimums[nIter], aNewMaximums[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}

/******************************************************************************
 * @brief Test the functionality of the InputAngleModulus function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-19
 ******************************************************************************/
TEST(NumOpsTest, InputAngleModulus)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength               = 9;
    const double aValues[nTestValuesLength]   = {1.0, -1.0, 4.0, 360.0, 350.0, 170.0, -90, 360.0, -90};
    const double aMinimums[nTestValuesLength] = {0.0, 0.0, -2.0, -180.0, -180.0, -180.0, 0.0, 0.0, 0.0};
    const double aMaximums[nTestValuesLength] = {2.0, 2.0, 2.0, 180.0, 180.0, 180.0, 360.0, 360.0, 360.0};
    const double aOutput[nTestValuesLength]   = {1.0, 1.0, 0.0, 0.0, -10.0, 170.0, 270.0, 0.0, 270.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::InputAngleModulus(aValues[nIter], aMinimums[nIter], aMaximums[nIter]);
        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}

/******************************************************************************
 * @brief Test the functionality of the AngularDifference function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-03
 ******************************************************************************/
TEST(NumOpsTest, AngularDifference)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                   = 8;
    const double aFirstValues[nTestValuesLength]  = {0.0, 330.0, 30.0, 270.0, 60.0, 0.0, 170.0, 60.0};
    const double aSecondValues[nTestValuesLength] = {0.0, 30.0, 330.0, 180.0, 120.0, 360.0, 190.0, 90.0};
    const double aOutput[nTestValuesLength]       = {0.0, 60.0, -60.0, 90.0, 60.0, 0.0, 20.0, 30.0};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Calculate valid bounds.
        double dResult = numops::AngularDifference(aFirstValues[nIter], aSecondValues[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_EQ(dResult, aOutput[nIter]);
    }
}

/******************************************************************************
 * @brief Test the functionality of the CoordinateFrameRotate3D function.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-03
 ******************************************************************************/
TEST(NumOpsTest, CoordinateFrameRotate3D)
{
    // Create array for storing input and expect output values.
    const int nTestValuesLength                = 8;
    const double aX[nTestValuesLength]         = {607350.5439843171, 15.0, 1.0, 1.0, 1.0, 5.0, 10.0, 420000.0};
    const double aY[nTestValuesLength]         = {314.26934576034546, 10.0, 0.0, 0.0, 0.0, 5.0, 10.0, 315.0};
    const double aZ[nTestValuesLength]         = {4201167.977962418, 40.0, 0.0, 0.0, 0.0, 5.0, 10.0, -65780.0};
    const double aXRot[nTestValuesLength]      = {0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 90.0, 0.0};
    const double aYRot[nTestValuesLength]      = {0.0, 0.0, 0.0, 90.0, 0.0, 90.0, 45.0, 65.4};
    const double aZRot[nTestValuesLength]      = {0.0, 0.0, 0.0, 0.0, 90.0, 90.0, 30.0, 0.0};
    const double aExpectedX[nTestValuesLength] = {607350.5439843171, 15.0, 1.0, 0.0, 0.0, 5.0, 17.25, 115028.38};
    const double aExpectedY[nTestValuesLength] = {314.2693457603454, 10.0, 0.0, 0.0, 1.0, 5.0, -1.59, 315.0};
    const double aExpectedZ[nTestValuesLength] = {4201167.977962418, 40.0, 0.0, -1.0, 0.0, -5.0, 0.0, -409262.11};

    // Loop through each value and compare inputs and outputs.
    for (int nIter = 0; nIter < nTestValuesLength; ++nIter)
    {
        // Pack point cloud vector.
        std::vector<numops::CoordinatePoint<double>> vPointCloud;
        vPointCloud.emplace_back(numops::CoordinatePoint(aX[nIter], aY[nIter], aZ[nIter]));

        // Calculate new rotated point.
        numops::CoordinateFrameRotate3D(vPointCloud, aXRot[nIter], aYRot[nIter], aZRot[nIter]);

        // Check that the expected output values were calculated.
        EXPECT_NEAR(vPointCloud[0].tX, aExpectedX[nIter], 0.01);
        EXPECT_NEAR(vPointCloud[0].tY, aExpectedY[nIter], 0.01);
        EXPECT_NEAR(vPointCloud[0].tZ, aExpectedZ[nIter], 0.01);
    }
}

/******************************************************************************
 * @brief Test the functionality of the CoordinatePoint structure
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
TEST(NumOpsTest, CoordinatePoint)
{
    // Default Constructor Test
    numops::CoordinatePoint<double> stDefaultPoint;
    EXPECT_DOUBLE_EQ(stDefaultPoint.tX, 0.0);
    EXPECT_DOUBLE_EQ(stDefaultPoint.tY, 0.0);
    EXPECT_DOUBLE_EQ(stDefaultPoint.tZ, 0.0);

    // Parameterized Constructor Test
    numops::CoordinatePoint<double> stParamPoint(1.5, 2.5, 3.5);
    EXPECT_DOUBLE_EQ(stParamPoint.tX, 1.5);
    EXPECT_DOUBLE_EQ(stParamPoint.tY, 2.5);
    EXPECT_DOUBLE_EQ(stParamPoint.tZ, 3.5);

    // Test with Integer Type
    numops::CoordinatePoint<int> stIntPoint(1, 2, 3);
    EXPECT_EQ(stIntPoint.tX, 1);
    EXPECT_EQ(stIntPoint.tY, 2);
    EXPECT_EQ(stIntPoint.tZ, 3);

    // Test with Float Type
    numops::CoordinatePoint<float> stFloatPoint(1.1f, 2.2f, 3.3f);
    EXPECT_FLOAT_EQ(stFloatPoint.tX, 1.1f);
    EXPECT_FLOAT_EQ(stFloatPoint.tY, 2.2f);
    EXPECT_FLOAT_EQ(stFloatPoint.tZ, 3.3f);

    // Test Modifying Values
    numops::CoordinatePoint<double> stModPoint(1.0, 2.0, 3.0);
    stModPoint.tX = 4.0;
    stModPoint.tY = 5.0;
    stModPoint.tZ = 6.0;
    EXPECT_DOUBLE_EQ(stModPoint.tX, 4.0);
    EXPECT_DOUBLE_EQ(stModPoint.tY, 5.0);
    EXPECT_DOUBLE_EQ(stModPoint.tZ, 6.0);
}
