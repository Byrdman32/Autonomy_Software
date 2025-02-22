/******************************************************************************
 * @brief Defines and implements functions related to operations on location or orientation
 *      coordinate systems within the geoops namespace.
 *
 * @file GeospatialOperations.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef GEOSPATIAL_OPERATIONS_HPP
#define GEOSPATIAL_OPERATIONS_HPP

#include "../AutonomyLogging.h"

/// \cond
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <chrono>

/// \endcond

/******************************************************************************
 * @brief Namespace containing functions related to operations on global position number
 *      systems and other datatypes.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-09-30
 ******************************************************************************/
namespace geoops
{
    /////////////////////////////////////////
    // Declare public enums.
    /////////////////////////////////////////

    // This enum is used to store values for the waypoint type.
    enum class WaypointType
    {
        eNavigationWaypoint,
        eTagWaypoint,
        eMalletWaypoint,
        eWaterBottleWaypoint,
        eObjectWaypoint,    // Used to represent either Mallet or WaterBottle waypoint.
        eObstacleWaypoint,
        eUNKNOWN
    };

    // This enum is used to store the location fix type for a UTM or GPS points, the enum indexes follow the UBLOX-F9P docs.
    enum class PositionFixType
    {
        eNoFix,
        eDeadReckoning,
        eFix2D,
        eFix3D,
        eGNSSDeadReckoningCombined,
        eTimeOnly,
        eUNKNOWN
    };

    /////////////////////////////////////////
    // Declare public variables.
    /////////////////////////////////////////

    /////////////////////////////////////////
    // Declare public structs that are specific to and used within this class.
    /////////////////////////////////////////

    /******************************************************************************
     * @brief This struct is used to store the distance, arc length, and relative bearing for a calculated
     *      geodesic between two points. Storing these values in a struct allows for easy
     *      handling and access to said variables.
     *
     * @note The arc length degree measurement is always from the first to second point.
     *      The relative bearing measurements are always CW positive with North being zero.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-13
     ******************************************************************************/
    struct GeoMeasurement
    {
        public:
            // Define public struct attributes.
            double dDistanceMeters;          // The great circle distance between the two points.
            double dArcLengthDegrees;        // The degree measurement difference between the two points from the center of the sphere. (0, 180)
            double dStartRelativeBearing;    // The relative bearing in degrees from the first point to the second point for the shortest great circle path. (0, 360)
            double dEndRelativeBearing;      // The relative bearing in degrees from the second point to the first point for the shortest great circle path. (0, 360)
    };

    /******************************************************************************
     * @brief This struct stores/contains information about orientation.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-23
     ******************************************************************************/
    struct IMUData
    {
        public:
            // Declare struct public member variables.
            double dPitch;
            double dRoll;
            double dHeading;

            /******************************************************************************
             * @brief Construct a new IMUData object.
             *
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            IMUData()
            {
                // Initialize member variables to default values.
                dPitch   = 0.0;
                dRoll    = 0.0;
                dHeading = 0.0;
            }

            /******************************************************************************
             * @brief Construct a new IMUData object.
             *
             * @param dPitch - The pitch of the navboard in degrees.
             * @param dRoll - The roll of the navboard in degrees.
             * @param dHeading - The bearing/yaw of the navboard in degrees.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            IMUData(double dPitch, double dRoll, double dHeading)
            {
                // Initialize member variables with given values.
                this->dPitch   = dPitch;
                this->dRoll    = dRoll;
                this->dHeading = dHeading;
            }
    };

    /******************************************************************************
     * @brief This struct stores/contains information about a GPS data.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-23
     ******************************************************************************/
    struct GPSCoordinate
    {
        public:
            // Declare struct public attributes
            double dLatitude;                              // The geographic latitude in degrees, typically within the range [-90, 90].
            double dLongitude;                             // The geographic longitude in degrees, typically within the range [-180, 180].
            double dAltitude;                              // The elevation above sea level in meters.
            double d2DAccuracy;                            // The horizontal accuracy of the GPS coordinates in meters.
            double d3DAccuracy;                            // The three-dimensional accuracy, including altitude, in meters.
            double dMeridianConvergence;                   // The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
            double dScale;                                 // The scale factor applied to the UTM coordinates for map projection.
            PositionFixType eCoordinateAccuracyFixType;    // The type of satellite location lock the coordinate was taken under.
            bool bIsDifferential;                          // Whether of not the coordinate was taken under a differential GPS lock.
            std::chrono::system_clock::time_point tmTimestamp;    // The chrono time point when the GPS coordinate was created/received.

            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////
            /******************************************************************************
             * @brief Construct a new GPSCoordinate object.
             *
             * @param dLatitude - The geographic latitude in degrees, typically within the range [-90, 90].
             * @param dLongitude - The geographic longitude in degrees, typically within the range [-180, 180].
             * @param dAltitude - The elevation above sea level in meters.
             * @param d2DAccuracy - The horizontal accuracy of the GPS coordinates in meters.
             * @param d3DAccuracy - The three-dimensional accuracy, including altitude, in meters.
             * @param dMeridianConvergence - The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
             * @param dScale - The scale factor applied to the UTM coordinates for map projection.
             * @param eCoordinateAccuracyFixType - The overall accuracy of the point, the satellite lock type the coordinate was taken under.
             * @param bIsDifferential - Whether of not the coordinate was taken under a differential GPS lock.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            GPSCoordinate(double dLatitude                                  = 0.0,
                          double dLongitude                                 = 0.0,
                          double dAltitude                                  = 0.0,
                          double d2DAccuracy                                = -1.0,
                          double d3DAccuracy                                = -1.0,
                          double dMeridianConvergence                       = -1.0,
                          double dScale                                     = 0.0,
                          PositionFixType eCoordinateAccuracyFixType        = PositionFixType::eUNKNOWN,
                          bool bIsDifferential                              = false,
                          std::chrono::system_clock::time_point tmTimestamp = std::chrono::system_clock::time_point().min())
            {
                // Initialize member variables with given values.
                this->dLatitude                  = dLatitude;
                this->dLongitude                 = dLongitude;
                this->dAltitude                  = dAltitude;
                this->d2DAccuracy                = d2DAccuracy;
                this->d3DAccuracy                = d3DAccuracy;
                this->dMeridianConvergence       = dMeridianConvergence;
                this->dScale                     = dScale;
                this->eCoordinateAccuracyFixType = eCoordinateAccuracyFixType;
                this->bIsDifferential            = bIsDifferential;
                this->tmTimestamp                = tmTimestamp;
            }

            /******************************************************************************
             * @brief Overridden operator equals for GPSCoordinate struct.
             *
             * @param stOtherCoordinate - The other GPSCoordinate struct we are comparing to.
             * @return true - The two GPSCoordinates are equal.
             * @return false - The two GPSCoordinates are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-04
             ******************************************************************************/
            bool operator==(const GPSCoordinate& stOtherCoordinate) const
            {
                // Check if location, altitude, and accuracy are the same. Not going to worry about other values for now.
                return (dLatitude == stOtherCoordinate.dLatitude && dLongitude == stOtherCoordinate.dLongitude && dAltitude == stOtherCoordinate.dAltitude &&
                        d2DAccuracy == stOtherCoordinate.d2DAccuracy && d3DAccuracy == stOtherCoordinate.d3DAccuracy &&
                        eCoordinateAccuracyFixType == stOtherCoordinate.eCoordinateAccuracyFixType && bIsDifferential == stOtherCoordinate.bIsDifferential);
            }

            /******************************************************************************
             * @brief Overridden operator not equals for GPSCoordinate struct.
             *
             * @param stOtherCoordinate - The other GPSCoordinate struct we are comparing to.
             * @return true - The two GPSCoordinates are not equal.
             * @return false - The two GPSCoordinates are equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-04
             ******************************************************************************/
            bool operator!=(const GPSCoordinate& stOtherCoordinate) const { return !this->operator==(stOtherCoordinate); }
    };

    /******************************************************************************
     * @brief This struct stores/contains information about a UTM coordinate.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-23
     ******************************************************************************/
    struct UTMCoordinate
    {
        public:
            // Declare struct public attributes.
            double dEasting;                               // The eastward displacement from the UTM zone's central meridian in meters.
            double dNorthing;                              // The northward displacement from the equator in meters.
            int nZone;                                     // The UTM zone number identifying the region on the Earth's surface.
            bool bWithinNorthernHemisphere;                // Indicates whether the coordinate is located in the northern hemisphere.
            double dAltitude;                              // The elevation above sea level in meters.
            double d2DAccuracy;                            // The horizontal accuracy of the UTM coordinates in meters.
            double d3DAccuracy;                            // The three-dimensional accuracy, including altitude, in meters.
            double dMeridianConvergence;                   // The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
            double dScale;                                 // The scale factor applied to the UTM coordinates for map projection.
            PositionFixType eCoordinateAccuracyFixType;    // The type of satellite location lock the coordinate was taken under.
            bool bIsDifferential;                          // Whether of not the coordinate was taken under a differential GPS lock.
            std::chrono::system_clock::time_point tmTimestamp;    // The chrono time point when the GPS coordinate was created/received.

            /////////////////////////////////////////
            // Declare public methods.
            /////////////////////////////////////////

            /******************************************************************************
             * @brief Construct a new UTMCoordinate object.
             *
             * @param dEasting - The eastward displacement from the UTM zone's central meridian in meters.
             * @param dNorthing - The northward displacement from the equator in meters.
             * @param nZone - The UTM zone number identifying the region on the Earth's surface.
             * @param bWithinNorthernHemisphere - Indicates whether the coordinate is located in the northern hemisphere.
             * @param dAltitude - The elevation above sea level in meters.
             * @param d2DAccuracy - The horizontal accuracy of the UTM coordinates in meters.
             * @param d3DAccuracy - The three-dimensional accuracy, including altitude, in meters.
             * @param dMeridianConvergence - The angle between true north and grid north at the given location in degrees. Positive in eastern direction.
             * @param dScale - The scale factor applied to the UTM coordinates for map projection.
             * @param eCoordinateAccuracyFixType - The overall accuracy of the point, the satellite lock type the coordinate was taken under.
             * @param bIsDifferential - Whether of not the coordinate was taken under a differential GPS lock.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2023-09-23
             ******************************************************************************/
            UTMCoordinate(double dEasting                                   = 0.0,
                          double dNorthing                                  = 0.0,
                          int nZone                                         = 0,
                          bool bWithinNorthernHemisphere                    = true,
                          double dAltitude                                  = 0.0,
                          double d2DAccuracy                                = -1.0,
                          double d3DAccuracy                                = -1.0,
                          double dMeridianConvergence                       = -1.0,
                          double dScale                                     = 0.0,
                          PositionFixType eCoordinateAccuracyFixType        = PositionFixType::eUNKNOWN,
                          bool bIsDifferential                              = false,
                          std::chrono::system_clock::time_point tmTimestamp = std::chrono::system_clock::time_point().min())
            {
                // Initialize member variables with given values.
                this->dEasting                   = dEasting;
                this->dNorthing                  = dNorthing;
                this->nZone                      = nZone;
                this->bWithinNorthernHemisphere  = bWithinNorthernHemisphere;
                this->dAltitude                  = dAltitude;
                this->d2DAccuracy                = d2DAccuracy;
                this->d3DAccuracy                = d3DAccuracy;
                this->dMeridianConvergence       = dMeridianConvergence;
                this->dScale                     = dScale;
                this->eCoordinateAccuracyFixType = eCoordinateAccuracyFixType;
                this->bIsDifferential            = bIsDifferential;
                this->tmTimestamp                = tmTimestamp;
            }

            /******************************************************************************
             * @brief Overridden operator equals for UTMCoordinate struct.
             *
             * @param stOtherCoordinate - The other UTMCoordinate struct we are comparing to.
             * @return true - The two UTMCoordinates are equal.
             * @return false - The two UTMCoordinates are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-04
             ******************************************************************************/
            bool operator==(const UTMCoordinate& stOtherCoordinate) const
            {
                // Check if location, altitude, and accuracy are the same. Not going to worry about other values for now.
                return (dEasting == stOtherCoordinate.dEasting && dNorthing == stOtherCoordinate.dNorthing && nZone == stOtherCoordinate.nZone &&
                        bWithinNorthernHemisphere == stOtherCoordinate.bWithinNorthernHemisphere && dAltitude == stOtherCoordinate.dAltitude &&
                        d2DAccuracy == stOtherCoordinate.d2DAccuracy && d3DAccuracy == stOtherCoordinate.d3DAccuracy &&
                        eCoordinateAccuracyFixType == stOtherCoordinate.eCoordinateAccuracyFixType && bIsDifferential == stOtherCoordinate.bIsDifferential);
            }

            /******************************************************************************
             * @brief Overridden operator not equals for UTMCoordinate struct.
             *
             * @param stOtherCoordinate - The other UTMCoordinate struct we are comparing to.
             * @return true - The two UTMCoordinates are not equal.
             * @return false - The two UTMCoordinates are equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-04
             ******************************************************************************/
            bool operator!=(const UTMCoordinate& stOtherCoordinate) const { return !this->operator==(stOtherCoordinate); }
    };

    /******************************************************************************
     * @brief Given a GPS coordinate, convert to UTM and create a new UTMCoordinate object.
     *
     * @param stGPSCoord - The struct containing the GPS coordinate data.
     * @return UTMCoordinate - The UTM coordinate corresponding to the given GPS coordinate.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-12
     ******************************************************************************/
    inline UTMCoordinate ConvertGPSToUTM(const GPSCoordinate& stGPSCoord)
    {
        // Create instance variables.
        UTMCoordinate stConvertCoord;

        // Get data out of the GPS coord and repackage it into the UTM struct.
        stConvertCoord.d2DAccuracy                = stGPSCoord.d2DAccuracy;
        stConvertCoord.d3DAccuracy                = stGPSCoord.d3DAccuracy;
        stConvertCoord.dAltitude                  = stGPSCoord.dAltitude;
        stConvertCoord.eCoordinateAccuracyFixType = stGPSCoord.eCoordinateAccuracyFixType;
        stConvertCoord.bIsDifferential            = stGPSCoord.bIsDifferential;
        stConvertCoord.tmTimestamp                = stGPSCoord.tmTimestamp;

        // Catch errors from GeographicLib.
        try
        {
            // Forward solve for the UTM coord.
            GeographicLib::UTMUPS::Forward(stGPSCoord.dLatitude,
                                           stGPSCoord.dLongitude,
                                           stConvertCoord.nZone,
                                           stConvertCoord.bWithinNorthernHemisphere,
                                           stConvertCoord.dEasting,
                                           stConvertCoord.dNorthing,
                                           stConvertCoord.dMeridianConvergence,
                                           stConvertCoord.dScale);
        }
        catch (const GeographicLib::GeographicErr::exception& geError)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Unable to forward solve a GPSCoordinate to UTMCoordinate. GeographicLib error is: {}", geError.what());
        }

        // Return the converted UTM coordinate.
        return stConvertCoord;
    }

    /******************************************************************************
     * @brief Given a UTM coordinate, convert to GPS and create a new GPSCoordinate object.
     *
     * @param stUTMCoord - The struct containing the UTM coordinate data.
     * @return GPSCoordinate - The GPS coordinate corresponding to the given UTM coordinate.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-12
     ******************************************************************************/
    inline GPSCoordinate ConvertUTMToGPS(const UTMCoordinate& stUTMCoord)
    {
        // Create instance variables.
        GPSCoordinate stConvertCoord;

        // Get data out of the UTM coord and repackage it into the GPS struct.
        stConvertCoord.d2DAccuracy                = stUTMCoord.d2DAccuracy;
        stConvertCoord.d3DAccuracy                = stUTMCoord.d3DAccuracy;
        stConvertCoord.dAltitude                  = stUTMCoord.dAltitude;
        stConvertCoord.eCoordinateAccuracyFixType = stUTMCoord.eCoordinateAccuracyFixType;
        stConvertCoord.bIsDifferential            = stUTMCoord.bIsDifferential;
        stConvertCoord.tmTimestamp                = stUTMCoord.tmTimestamp;

        // Catch errors from GeographicLib.
        try
        {
            // Reverse solve for the UTM coord.
            GeographicLib::UTMUPS::Reverse(stUTMCoord.nZone,
                                           stUTMCoord.bWithinNorthernHemisphere,
                                           std::fabs(stUTMCoord.dEasting),
                                           stUTMCoord.dNorthing,
                                           stConvertCoord.dLatitude,
                                           stConvertCoord.dLongitude,
                                           stConvertCoord.dMeridianConvergence,
                                           stConvertCoord.dScale);
        }
        catch (const GeographicLib::GeographicErr::exception& geError)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "Unable to reverse solve a UTMCoordinate to GPSCoordinate. GeographicLib error is: {}", geError.what());
        }

        // Return the converted UTM coordinate.
        return stConvertCoord;
    }

    /******************************************************************************
     * @brief The shortest path between two points on an ellipsoid at (lat1, lon1) and (lat2, lon2) is called the geodesic.
     *      Given those two points create an ellipsoid with earth's characteristics and find the distance between them.
     *
     * @param stCoord1 - The first GPS coordinate.
     * @param stCoord2 - The second GPS coordinate.
     * @return GeoMeasurement - Struct containing the distance in meters and arc length degrees, plus the bearing relative to the first point and second point.
     *
     * @see https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1Geodesic.html#ae66c9cecfcbbcb1da52cb408e69f65de
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-13
     ******************************************************************************/
    inline GeoMeasurement CalculateGeoMeasurement(const GPSCoordinate& stCoord1, const GPSCoordinate& stCoord2)
    {
        // Create instance variables.
        GeoMeasurement stMeasurements;

        // Construct a geodesic with earth characteristics. (Radius and flattening)
        // The WGS84 standard is widely used and aligns with Google Maps.
        GeographicLib::Geodesic geGeodesic = GeographicLib::Geodesic::WGS84();

        // Solve the inverse geodesic for distance and arc length degrees at the center of the globe, and relative bearings.
        stMeasurements.dArcLengthDegrees = geGeodesic.Inverse(stCoord1.dLatitude,
                                                              stCoord1.dLongitude,
                                                              stCoord2.dLatitude,
                                                              stCoord2.dLongitude,
                                                              stMeasurements.dDistanceMeters,
                                                              stMeasurements.dStartRelativeBearing,
                                                              stMeasurements.dEndRelativeBearing);

        // NOTE: Regarding azi1 vs azi2, azi1 is the direction measured at point 1 (your navigation aid) to point 2. azi2 is the direction measured at point 2 (your
        // location) away from point 1. (If you want the direction to point 1, add ±180° to azi2.)
        // Map the -180, 180 range of the azimuths to 0, 360, with both points zeroed at North.
        stMeasurements.dStartRelativeBearing = std::fmod((stMeasurements.dStartRelativeBearing + 360), 360);
        stMeasurements.dEndRelativeBearing   = std::fmod((stMeasurements.dEndRelativeBearing + 180), 360);
        // Ensure the result angle is positive.
        if (stMeasurements.dStartRelativeBearing < 0)
        {
            // Add 360 degrees.
            stMeasurements.dStartRelativeBearing += 360;
        }
        // Ensure the result angle is positive.
        if (stMeasurements.dEndRelativeBearing < 0)
        {
            // Add 360 degrees.
            stMeasurements.dEndRelativeBearing += 360;
        }

        // Return result distance.
        return stMeasurements;
    }

    /******************************************************************************
     * @brief The shortest path between two points on an ellipsoid at (easting1, northing1) and (easting2, northing2) is called the geodesic.
     *      Given those two points create an ellipsoid with earth's characteristics and find the distance between them.
     *
     * @param stCoord1 - The first UTM coordinate.
     * @param stCoord2 - The second UTM coordinate.
     * @return GeoMeasurement - Struct containing the distance in meters and arc length degrees, plus the bearing relative to the first point and second point.
     *
     * @see https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1Geodesic.html#ae66c9cecfcbbcb1da52cb408e69f65de
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-01-14
     ******************************************************************************/
    inline GeoMeasurement CalculateGeoMeasurement(const UTMCoordinate& stCoord1, const UTMCoordinate& stCoord2)
    {
        // Create instance variables.
        GeoMeasurement stMeasurements;

        // Construct a geodesic with earth characteristics. (Radius and flattening)
        // The WGS84 standard is widely used and aligns with Google Maps.
        GeographicLib::Geodesic geGeodesic = GeographicLib::Geodesic::WGS84();

        // Convert the given UTM coords into GPS coords for temporary use.
        GPSCoordinate stGPSCoord1 = ConvertUTMToGPS(stCoord1);
        GPSCoordinate stGPSCoord2 = ConvertUTMToGPS(stCoord2);

        // Solve the inverse geodesic.
        stMeasurements.dArcLengthDegrees = geGeodesic.Inverse(stGPSCoord1.dLatitude,
                                                              stGPSCoord1.dLongitude,
                                                              stGPSCoord2.dLatitude,
                                                              stGPSCoord2.dLongitude,
                                                              stMeasurements.dDistanceMeters,
                                                              stMeasurements.dStartRelativeBearing,
                                                              stMeasurements.dEndRelativeBearing);

        // NOTE: Regarding azi1 vs azi2, azi1 is the direction measured at point 1 (your navigation aid) to point 2. azi2 is the direction measured at point 2 (your
        // location) away from point 1. (If you want the direction to point 1, add ±180° to azi2.)
        // Map the -180, 180 range of the azimuths to 0, 360, with both points zeroed at North.
        stMeasurements.dStartRelativeBearing = std::fmod((stMeasurements.dStartRelativeBearing + 360), 360);
        stMeasurements.dEndRelativeBearing   = std::fmod((stMeasurements.dEndRelativeBearing + 180), 360);
        // Ensure the result angle is positive.
        if (stMeasurements.dStartRelativeBearing < 0)
        {
            // Add 360 degrees.
            stMeasurements.dStartRelativeBearing += 360;
        }
        // Ensure the result angle is positive.
        if (stMeasurements.dEndRelativeBearing < 0)
        {
            // Add 360 degrees.
            stMeasurements.dEndRelativeBearing += 360;
        }

        // Return result distance.
        return stMeasurements;
    }

    /******************************************************************************
     * @brief This struct is used by the WaypointHandler class to store location, size,
     *      and type information about a given location of interest of waypoint.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-02-02
     ******************************************************************************/
    struct Waypoint
    {
        private:
            // Declare struct private member variables.
            geoops::GPSCoordinate stGPSLocation;    // This is not meant to be changed once this waypoint is created.
            geoops::UTMCoordinate stUTMLocation;    // This is not meant to be changed once this waypoint is created.

        public:
            // Declare struct public member variables.
            WaypointType eType;
            double dRadius;
            int nID;

            /******************************************************************************
             * @brief Construct a new Waypoint object.
             *
             * @param stGPSLocation - The location of this waypoint stored in a geoops namespace
             *                  GPSCoordinate struct.
             * @param eType - The waypoint type. Navigation, Intermediate, Tag, Object, etc.
             * @param dRadius - The size of the waypoint. This is mainly only useful for objects or when
             *              you want the rover to just go to a general area.
             *
             * @note This will also store the equivalent UTM coordinate.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-02
             ******************************************************************************/
            Waypoint(const geoops::GPSCoordinate& stGPSLocation = geoops::GPSCoordinate(),
                     const WaypointType& eType                  = WaypointType::eUNKNOWN,
                     const double dRadius                       = 0.0,
                     const int nID                              = -1)
            {
                // Initialize member variables.
                this->stGPSLocation = stGPSLocation;
                this->stUTMLocation = geoops::ConvertGPSToUTM(stGPSLocation);
                this->eType         = eType;
                this->dRadius       = dRadius;
                this->nID           = nID;
            }

            /******************************************************************************
             * @brief Construct a new Waypoint object.
             *
             * @param stUTMLocation - The location of this waypoint stored in a geoops namespace
             *                  UTMCoordinate struct.
             * @param eType - The waypoint type. Navigation, Intermediate, Tag, Object, etc.
             * @param dRadius - The size of the waypoint. This is mainly only useful for objects or when
             *              you want the rover to just go to a general area.
             *
             * @note This will also store the equivalent GPS coordinate.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-02
             ******************************************************************************/
            Waypoint(const geoops::UTMCoordinate& stUTMLocation, const WaypointType& eType = WaypointType::eUNKNOWN, const double dRadius = 0.0, const int nID = -1)
            {
                // Initialize member variables.
                this->stUTMLocation = stUTMLocation;
                this->stGPSLocation = geoops::ConvertUTMToGPS(stUTMLocation);
                this->eType         = eType;
                this->dRadius       = dRadius;
                this->nID           = nID;
            }

            /******************************************************************************
             * @brief Accessor for the geoops::GPSCoordinate member variable.
             *
             * @return geoops::GPSCoordinate - The location of the waypoint stored in a geoops namespace
             *                      GPSCoordinate struct.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-02
             ******************************************************************************/
            const geoops::GPSCoordinate& GetGPSCoordinate() const { return stGPSLocation; }

            /******************************************************************************
             * @brief Accessor for the geoops::UTMCoordinate member variable.
             *
             * @return geoops::UTMCoordinate - The location of the waypoint stored in a geoops namespace
             *                      UTMCoordinate struct.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-02
             ******************************************************************************/
            const geoops::UTMCoordinate& GetUTMCoordinate() const { return stUTMLocation; }

            /******************************************************************************
             * @brief Overridden operator equals for Waypoint struct.
             *
             * @param stOtherWaypoint - The other Waypoint struct we are comparing to.
             * @return true - The two Waypoints are equal.
             * @return false - The two Waypoints are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-02-04
             ******************************************************************************/
            bool operator==(const Waypoint& stOtherWaypoint) const
            {
                // Check if location, altitude, and accuracy are the same. Not going to worry about other values for now.
                return (stGPSLocation == stOtherWaypoint.stGPSLocation && stUTMLocation == stOtherWaypoint.stUTMLocation && eType == stOtherWaypoint.eType &&
                        dRadius == stOtherWaypoint.dRadius && nID == stOtherWaypoint.nID);
            }

            /******************************************************************************
             * @brief Overridden operator equals for Waypoint struct.
             *
             * @param stOtherWaypoint - The other Waypoint struct we are comparing to.
             * @return true - The two Waypoints are equal.
             * @return false - The two Waypoints are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-03-18
             ******************************************************************************/
            bool operator!=(const Waypoint& stOtherWaypoint) const { return !this->operator==(stOtherWaypoint); }
    };

    /******************************************************************************
     * @brief This struct is used by the WaypointHandler to provide an easy way to store
     *      all pose data about the rover.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-04-08
     ******************************************************************************/
    struct RoverPose
    {
        private:
            // Declare struct private member variables.
            Waypoint stRoverPosition;    // Stores the rover's 3D location in UTM and GPS form.
            double dRoverHeading;        // Stores the rover's compass heading. North = 0, CW=pos

        public:
            /******************************************************************************
             * @brief Construct a new Rover Pose object.
             *
             * @param stRoverPosition - The current location of the rover stored in a GPSCoordinate struct.
             * @param dRoverHeading - The current heading of the rover.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-08
             ******************************************************************************/
            RoverPose(const geoops::GPSCoordinate& stRoverPosition = geoops::GPSCoordinate(), const double dRoverHeading = 0.0)
            {
                // Update member variables.
                this->stRoverPosition = Waypoint(stRoverPosition);
                this->dRoverHeading   = dRoverHeading;
            }

            /******************************************************************************
             * @brief Construct a new Rover Pose object.
             *
             * @param stRoverPosition - The current location of the rover stored in a UTMCoordinate struct.
             * @param dRoverHeading - The current heading of the rover.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-08
             ******************************************************************************/
            RoverPose(const geoops::UTMCoordinate& stRoverPosition, const double dRoverHeading = 0.0)
            {
                // Update member variables.
                this->stRoverPosition = Waypoint(stRoverPosition);
                this->dRoverHeading   = dRoverHeading;
            }

            /******************************************************************************
             * @brief Accessor for the geoops::GPSCoordinate member variable.
             *
             * @return geoops::GPSCoordinate - The location of the RoverPose stored in a geoops namespace
             *                      GPSCoordinate struct.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-08
             ******************************************************************************/
            const geoops::GPSCoordinate& GetGPSCoordinate() const { return stRoverPosition.GetGPSCoordinate(); }

            /******************************************************************************
             * @brief Accessor for the geoops::UTMCoordinate member variable.
             *
             * @return geoops::UTMCoordinate - The location of the RoverPose stored in a geoops namespace
             *                      UTMCoordinate struct.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-08
             ******************************************************************************/
            const geoops::UTMCoordinate& GetUTMCoordinate() const { return stRoverPosition.GetUTMCoordinate(); }

            /******************************************************************************
             * @brief Accessor for the Compass Heading private member.
             *
             * @return const double - The compass heading of the rover pose.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-08
             ******************************************************************************/
            double GetCompassHeading() const { return dRoverHeading; }

            /******************************************************************************
             * @brief Overridden operator equals for RoverPose struct.
             *
             * @param stOtherRoverPose - The other Waypoint struct we are comparing to.
             * @return true - The two RoverPoses are equal.
             * @return false - The two RoverPoses are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-08
             ******************************************************************************/
            bool operator==(const RoverPose& stOtherRoverPose) const
            {
                // Check if location, altitude, and accuracy are the same. Not going to worry about other values for now.
                return (stRoverPosition == stOtherRoverPose.GetGPSCoordinate() && stRoverPosition == stOtherRoverPose.GetUTMCoordinate() &&
                        dRoverHeading == stOtherRoverPose.dRoverHeading);
            }

            /******************************************************************************
             * @brief Overridden operator equals for RoverPose struct.
             *
             * @param stOtherRoverPose - The other RoverPose struct we are comparing to.
             * @return true - The two RoverPoses are equal.
             * @return false - The two RoverPoses are not equal.
             *
             * @author clayjay3 (claytonraycowen@gmail.com)
             * @date 2024-04-08
             ******************************************************************************/
            bool operator!=(const RoverPose& stOtherRoverPose) const { return !this->operator==(stOtherRoverPose); }
    };
}    // namespace geoops
#endif
