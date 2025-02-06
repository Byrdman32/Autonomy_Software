/******************************************************************************
 * @brief Unit test for the Idle State.
 *
 * @file IdleState.cc
 * @author Lucas Wiley (lrw9my@mst.edu)
 * @date 2024-11-21
 *
 * @copyright Copyright MRDT 2025 - All Rights Reserved
 ******************************************************************************/

// #include "../../../../states/IdleState.h"
#include "../../../../src/states/IdleState.cpp"
#include "../../../TestingBase.hh"

/// \cond
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the Idle State
 *
 * @author Bailey Schoenike (bpsrdf@umsystem.edu)
 * @date 2025-02-06
 ******************************************************************************/
class IdleStateTests : public TestingBase<IdleStateTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new Idle State Tests object.
         *
         * @author Bailey Schoenike (bpsrdf@umsystem.edu)
         * @date 2025-02-06
         ******************************************************************************/
        IdleStateTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the Idle State Tests object.
         *
         * @author Bailey Schoenike (bpsrdf@umsystem.edu)
         * @date 2025-02-06
         ******************************************************************************/
        ~IdleStateTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the Idle State Tests object.
         *
         * @author Bailey Schoenike (bpsrdf@umsystem.edu)
         * @date 2025-02-06
         ******************************************************************************/
        void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
        }

        /******************************************************************************
         * @brief Teardown the Idle State Tests object.
         *
         * @author Bailey Schoenike (bpsrdf@umsystem.edu)
         * @date 2025-02-06
         ******************************************************************************/
        void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
        }
};

/******************************************************************************
 * @brief Unit Test Class for the Example Tests
 *
 * @author Bailey Schoenike (bpsrdf@umsystem.edi)
 * @date 2025-02-06
 ******************************************************************************/
TEST_F(IdleStateTests, temp) {}
