/******************************************************************************
 * @brief Unit test for SearchPattern algorithm namespace.
 *
 * @file SearchPattern.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/algorithms/SearchPattern.hpp"
#include "../../../../src/util/GeospatialOperations.hpp"
#include "../../../TestingBase.hh"

/// \cond
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the SearchPattern
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class SearchPatternTests : public TestingBase<SearchPatternTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

        /******************************************************************************
         * @brief Function used in testing to determine if a returns list of waypoints
         *      is a good spiral.
         *
         * @param vPoints - The list of waypoint that, in order, should form a spiral.
         * @return true - The list of waypoints is a valid spiral.
         * @return false - The list of waypoint is not a valid spiral.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-01
         ******************************************************************************/
        bool IsOutwardSpiral(const std::vector<geoops::Waypoint>& vPoints)
        {
            // At least 4 vPoints are needed to form a spiral.
            if (vPoints.size() < 4)
            {
                return false;
            }

            // Store starting point.
            geoops::UTMCoordinate stCenterPoint = vPoints[0].GetUTMCoordinate();
            // Store geo distance information of last point.
            geoops::GeoMeasurement stLastGeodesicFromCenterPoint;

            // Loop through each waypoint.
            for (int nIter = 1; nIter < vPoints.size(); ++nIter)
            {
                // Check if this is the first iteration.
                if (nIter == 1)
                {
                    // Just calculate geo distance between start and first point and store it.
                    stLastGeodesicFromCenterPoint = geoops::CalculateGeoMeasurement(stCenterPoint, vPoints[nIter].GetUTMCoordinate());
                }
                else
                {
                    // Calculate geo measurement for new point.
                    geoops::GeoMeasurement stNewMeasurement = geoops::CalculateGeoMeasurement(stCenterPoint, vPoints[nIter].GetUTMCoordinate());

                    // Check that the measurement is further away from the center point that last time and is radially progressing in some direction.
                    if (stNewMeasurement.dDistanceMeters < stLastGeodesicFromCenterPoint.dDistanceMeters ||
                        stNewMeasurement.dStartRelativeBearing == stLastGeodesicFromCenterPoint.dStartRelativeBearing ||
                        stNewMeasurement.dEndRelativeBearing == stLastGeodesicFromCenterPoint.dEndRelativeBearing)
                    {
                        // Conditions net met, this is not a proper spiral.
                        return false;
                    }

                    // Update last measurement variable.
                    stLastGeodesicFromCenterPoint = stNewMeasurement;
                }
            }

            return true;
        }

        /******************************************************************************
         * @brief Function used in testing to determine if a returns list of waypoints
         *      is a good zigzag.
         *
         * @param vPoints - The list of waypoint that, in order, should form a zigzag.
         * @return true - The list of waypoints is a valid zigzag.
         * @return false - The list of waypoint is not a valid zigzag.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-04-01
         ******************************************************************************/
        bool IsZigZag(const std::vector<geoops::Waypoint>& vPoints, const double dSlopeThreshold = 0.1)
        {
            // Create instance variables.
            bool bZigZag          = false;
            double dExpectedSlope = 0.0;

            // At least 3 points are needed for a zigzag.
            if (vPoints.size() < 3)
            {
                return false;
            }

            // Loop through path points.
            for (size_t siIter = 1; siIter < vPoints.size(); ++siIter)
            {
                // Calculate slope of last two points.
                double dSlope = atan2(vPoints[siIter].GetUTMCoordinate().dNorthing - vPoints[siIter - 1].GetUTMCoordinate().dNorthing,
                                      vPoints[siIter].GetUTMCoordinate().dEasting - vPoints[siIter - 1].GetUTMCoordinate().dEasting);

                // Make sure this isn't the first iteration of the loop.
                if (siIter > 1)
                {
                    // Calculate the previous slope.
                    double dPrevSlope = atan2(vPoints[siIter - 1].GetUTMCoordinate().dNorthing - vPoints[siIter - 2].GetUTMCoordinate().dNorthing,
                                              vPoints[siIter - 1].GetUTMCoordinate().dEasting - vPoints[siIter - 2].GetUTMCoordinate().dEasting);
                    // Check if the slope has changed.
                    if (std::abs(dSlope - dPrevSlope) > dSlopeThreshold)
                    {
                        // Check if the slope is the same value as the dExpected slope and has the same sign.
                        if ((dSlope - dPrevSlope) == dExpectedSlope)
                        {
                            // Set this is a zigzag.
                            bZigZag = true;
                        }
                        else
                        {
                            return false;
                        }
                    }
                }
                // This is the firs iteration.
                if (siIter < vPoints.size() - 1)
                {
                    // Calculate the next slope.
                    double dNextSlope = atan2(vPoints[siIter + 1].GetUTMCoordinate().dNorthing - vPoints[siIter].GetUTMCoordinate().dNorthing,
                                              vPoints[siIter + 1].GetUTMCoordinate().dEasting - vPoints[siIter].GetUTMCoordinate().dEasting);
                    // Check if the slope has changed.
                    if (std::abs(dSlope - dNextSlope) > dSlopeThreshold)
                    {
                        // Store the next expected slope change. (will have opposite sign)
                        dExpectedSlope = -(dSlope - dNextSlope);
                    }
                }
            }

            // If either increasing or decreasing slope condition is met, it's a zigzag.
            return bZigZag;
        }

    public:
        /******************************************************************************
         * @brief Construct a new Search Pattern Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        SearchPatternTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the Search Pattern Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~SearchPatternTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the Search Pattern Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
        }

        /******************************************************************************
         * @brief Teardown the Search Pattern Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
        }
};

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST_F(SearchPatternTests, SpiralPatternShapeGPS)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stGPSRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsOutwardSpiral(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-10-12
 ******************************************************************************/
TEST_F(SearchPatternTests, SpiralPatternShapeUTM)
{
    // Create a new GPS coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stUTMRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsOutwardSpiral(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-01
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternShapeGPS)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternShapeGPSHorizontal)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate, 20.0, 20.0, 1.0, false);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2024-04-01
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternShapeUTM)
{
    // Create a new GPS coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternShapeUTMHorizontal)
{
    // Create a new GPS coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 20.0, 20.0, 1.0, false);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, SpiralPatternRadiusTooSmall)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateSpiralPatternWaypoints(stGPSRollaCoordinate, 57, 0);

    // Create a new UTm coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPathUTM = searchpattern::CalculateSpiralPatternWaypoints(stUTMRollaCoordinate, 57, 0);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(vSearchPatternPath.empty());
    EXPECT_TRUE(vSearchPatternPathUTM.empty());
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternRadiusTooSmall)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate, 20.0, 20.0, 0);

    // Create a new UTm coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPathUTM = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 20.0, 20.0, 0);

    // Check if the returned path resembles an outward spiral pattern.
    EXPECT_TRUE(vSearchPatternPath.empty());
    EXPECT_TRUE(vSearchPatternPathUTM.empty());
}

/******************************************************************************
 * @brief Test SearchPattern algorithm functionality.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-06
 ******************************************************************************/
TEST_F(SearchPatternTests, ZigZagPatternLimitSpacing)
{
    // Create a new GPS coordinate.
    geoops::GPSCoordinate stGPSRollaCoordinate(37.951766, -91.778187);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate, 20.0, 20.0, 30.0);
    // Create a new UTm coordinate.
    geoops::UTMCoordinate stUTMRollaCoordinate(607344.14, 4201167.33, 15, true);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPathUTM = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 20.0, 20.0, 30.0);

    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPath2 = searchpattern::CalculateZigZagPatternWaypoints(stGPSRollaCoordinate, 60.0, 20.0, 30.0);
    // Use this for generating a search pattern with default params.
    std::vector<geoops::Waypoint> vSearchPatternPathUTM2 = searchpattern::CalculateZigZagPatternWaypoints(stUTMRollaCoordinate, 60.0, 20.0, 30.0);

    // Both paths should be zigzags.
    EXPECT_TRUE(IsZigZag(vSearchPatternPath));
    EXPECT_TRUE(IsZigZag(vSearchPatternPathUTM));
    EXPECT_TRUE(IsZigZag(vSearchPatternPath2));
    EXPECT_TRUE(IsZigZag(vSearchPatternPathUTM2));
}
