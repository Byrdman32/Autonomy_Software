/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine.
 *
 * @file SearchPatternState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "SearchPatternState.h"
#include "../AutonomyGlobals.h"
#include "../algorithms/DifferentialDrive.hpp"
#include "../algorithms/SearchPattern.hpp"
#include "../interfaces/State.hpp"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief This method is called when the state is first started. It is used to
     *        initialize the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void SearchPatternState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_eCurrentSearchPatternType = eSpiral;
        m_nSearchPathIdx            = 0;
        m_stSearchPatternCenter     = globals::g_pWaypointHandler->PeekNextWaypoint();

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();

        // Calculate the search path.
        m_vSearchPath   = searchpattern::CalculateSpiralPatternWaypoints(m_stSearchPatternCenter.GetGPSCoordinate(),
                                                                       constants::SEARCH_ANGULAR_STEP_DEGREES,
                                                                       m_stSearchPatternCenter.dRadius,
                                                                       stCurrentRoverPose.GetCompassHeading(),
                                                                       constants::SEARCH_SPIRAL_SPACING);

        m_vTagDetectors = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                           globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameLeftCam),
                           globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameRightCam)};
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void SearchPatternState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Exiting state.");

        // Stop drive.
        globals::g_pDriveBoard->SendStop();
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    SearchPatternState::SearchPatternState() : State(States::eSearchPattern)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        // Initialize member variables.
        m_bInitialized  = false;
        m_StuckDetector = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                       constants::STUCK_CHECK_INTERVAL,
                                                                       constants::STUCK_CHECK_VEL_THRESH,
                                                                       constants::STUCK_CHECK_ROT_THRESH);
        // Start state.
        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author Jason Pittman (jspencerpittman@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void SearchPatternState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Running state-specific behavior.");

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();

        /*
            The overall flow of this state is as follows.
            1. Is there a tag -> MarkerSeen
            2. Is there an object -> ObjectSeen
            3. Is there an obstacle -> TBD
            4. Is the rover stuck -> Stuck
            5. Is the search pattern complete -> Abort
            6. Follow the search pattern.
        */

        /////////////////////////
        /* --- Detect Tags --- */
        /////////////////////////

        // Get a list of the currently detected tags, and their stats.
        std::vector<arucotag::ArucoTag> vDetectedArucoTags;
        std::vector<tensorflowtag::TensorflowTag> vDetectedTensorflowTags;
        tagdetectutils::LoadDetectedTags(vDetectedArucoTags, vDetectedTensorflowTags, m_vTagDetectors, false);

        // Check if we have detected any tags.
        if (vDetectedArucoTags.size() || vDetectedTensorflowTags.size())
        {
            // Check if any of the tags have a detection counter or confidence greater than the threshold.
            if (std::any_of(vDetectedArucoTags.begin(),
                            vDetectedArucoTags.end(),
                            [this](arucotag::ArucoTag& stTag)
                            {
                                // If the Tag ID given by the user in the waypoint is less than 0, then we don't care about the ID.
                                if (m_stSearchPatternCenter.nID < 0)
                                {
                                    return stTag.nHits >= constants::APPROACH_MARKER_DETECT_ATTEMPTS_LIMIT;
                                }
                                else
                                {
                                    return (stTag.nID == m_stSearchPatternCenter.nID && stTag.nHits >= constants::APPROACH_MARKER_DETECT_ATTEMPTS_LIMIT);
                                }
                            }) ||
                std::any_of(vDetectedTensorflowTags.begin(),
                            vDetectedTensorflowTags.end(),
                            [](tensorflowtag::TensorflowTag& stTag) { return stTag.dConfidence >= constants::APPROACH_MARKER_TF_CONFIDENCE_THRESHOLD; }))
            {
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: Marker seen!");
                // Handle state transition.
                globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerSeen, true);
                return;
            }
        }

        ////////////////////////////
        /* --- Detect Objects --- */
        ////////////////////////////

        // TODO: Add object detection to SearchPattern state

        //////////////////////////////
        /* --- Detect Obstacles --- */
        //////////////////////////////

        // TODO: Add obstacle detection to SearchPattern state

        //////////////////////////////////////////
        /* ---  Check if the rover is stuck --- */
        //////////////////////////////////////////

        // Check if stuck.
        if (m_StuckDetector.CheckIfStuck(globals::g_pWaypointHandler->SmartRetrieveVelocity(), globals::g_pWaypointHandler->SmartRetrieveAngularVelocity()))
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "SearchPattern: Rover has become stuck!");
            // Increment search path index so we skip the waypoint where we got stuck when reentering searchpattern.
            m_nSearchPathIdx += 1;
            // Check path index is within bounds.
            if (m_nSearchPathIdx >= int(m_vSearchPath.size()))
            {
                m_nSearchPathIdx = m_vSearchPath.size() - 1;
            }
            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
            // Don't execute the rest of the state.
            return;
        }

        ///////////////////////////////////
        /* --- Follow Search Pattern --- */
        ///////////////////////////////////

        // Check if the search path is empty.
        if (m_vSearchPath.empty())
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "SearchPatternState: Search path is empty, aborting search.");
            // Handle state transition.
            globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
            return;
        }

        // Have we reached the current waypoint?
        geoops::GPSCoordinate stCurrTargetGPS    = m_vSearchPath[m_nSearchPathIdx].GetGPSCoordinate();
        geoops::GeoMeasurement stCurrRelToTarget = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetGPSCoordinate(), stCurrTargetGPS);
        bool bReachedTarget                      = stCurrRelToTarget.dDistanceMeters <= constants::SEARCH_WAYPOINT_PROXIMITY;

        // If the entire search pattern has been completed without seeing tags or objects, try different search pattern.
        if (bReachedTarget && m_nSearchPathIdx >= int(m_vSearchPath.size() - 1))
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eSearchFailed);
            return;
        }
        // Move on to the next waypoint in the search path.
        else if (bReachedTarget)
        {
            ++m_nSearchPathIdx;
            stCurrTargetGPS   = m_vSearchPath[m_nSearchPathIdx].GetGPSCoordinate();
            stCurrRelToTarget = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetGPSCoordinate(), stCurrTargetGPS);
        }

        // Drive to target waypoint.
        diffdrive::DrivePowers stDrivePowers = globals::g_pDriveBoard->CalculateMove(constants::SEARCH_MOTOR_POWER,
                                                                                     stCurrRelToTarget.dStartRelativeBearing,
                                                                                     stCurrentRoverPose.GetCompassHeading(),
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive);
        globals::g_pDriveBoard->SendDrive(stDrivePowers);

        return;
    }

    /******************************************************************************
     * @brief Trigger an event in the state machine. Returns the next state.
     *
     * @param eEvent - The event to trigger.
     * @return std::shared_ptr<State> - The next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    States SearchPatternState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eSearchPattern;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eMarkerSeen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling MarkerSeen event.");
                // Change states.
                eNextState = States::eApproachingMarker;
                break;
            }
            case Event::eObjectSeen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling ObjectSeen event.");
                // Change state.
                eNextState = States::eApproachingObject;
                break;
            }
            case Event::eStart:
            {
                // Submit logger message
                LOG_NOTICE(logging::g_qSharedLogger, "SearchPatternState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eSearchFailed:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling SearchFailed event.");
                // Stop drive.
                globals::g_pDriveBoard->SendStop();

                // Regenerate a new search pattern.
                switch (m_eCurrentSearchPatternType)
                {
                    // Check which pattern to do next.
                    case eSpiral:
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "SearchPatternState: Spiral search pattern failed, trying vertical ZigZag...");
                        // Generate vertical zigzag pattern.
                        m_vSearchPath = searchpattern::CalculateZigZagPatternWaypoints(m_stSearchPatternCenter.GetGPSCoordinate(),
                                                                                       m_stSearchPatternCenter.dRadius * 2,
                                                                                       m_stSearchPatternCenter.dRadius * 2,
                                                                                       constants::SEARCH_ZIGZAG_SPACING,
                                                                                       true);
                        // Reset index counter.
                        m_nSearchPathIdx = 0;
                        // Update current search pattern
                        m_eCurrentSearchPatternType = eZigZag;
                        break;
                    }
                    case eZigZag:
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "SearchPatternState: Vertical ZigZag search pattern failed, trying horizontal ZigZag...");
                        // Generate vertical zigzag pattern.
                        m_vSearchPath = searchpattern::CalculateZigZagPatternWaypoints(m_stSearchPatternCenter.GetGPSCoordinate(),
                                                                                       m_stSearchPatternCenter.dRadius * 2,
                                                                                       m_stSearchPatternCenter.dRadius * 2,
                                                                                       constants::SEARCH_ZIGZAG_SPACING,
                                                                                       false);
                        // Reset index counter.
                        m_nSearchPathIdx = 0;
                        // Update current search pattern
                        m_eCurrentSearchPatternType = END;
                        break;
                    }
                    case END:
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "SearchPatternState: All patterns failed to find anything, giving up...");
                        // Pop old waypoint out of queue.
                        globals::g_pWaypointHandler->PopNextWaypoint();
                        // Change states.
                        eNextState = States::eIdle;
                        break;
                    }
                    default:
                    {
                        // Change states.
                        eNextState = States::eIdle;
                        break;
                    }
                }
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eStuck:
            {
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling Stuck event.");
                eNextState = States::eStuck;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "SearchPatternState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eSearchPattern)
        {
            LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
