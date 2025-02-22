/******************************************************************************
 * @brief Reversing State Implementation for Autonomy State Machine.
 *
 * @file ReversingState.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef REVERSING_STATE_H
#define REVERSING_STATE_H

#include "../interfaces/State.hpp"
#include "../util/GeospatialOperations.hpp"

/// \cond
#include <chrono>

/// \endcond

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief The ReversingState class implements the Reversing state for the Autonomy
     *        State Machine.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    class ReversingState : public State
    {
        private:
            geoops::RoverPose m_stStartRoverPose;
            std::chrono::system_clock::time_point m_tmStartReversingTime;
            std::chrono::system_clock::time_point m_tmTimeSinceLastMeter;
            bool m_bInitialized;

        protected:
            void Start() override;
            void Exit() override;

        public:
            ReversingState();
            void Run() override;
            States TriggerEvent(Event eEvent) override;
    };
}    // namespace statemachine

#endif    // REVERSINGSTATE_H
