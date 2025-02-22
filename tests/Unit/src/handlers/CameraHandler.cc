/******************************************************************************
 * @brief Unit test for CameraHandler handler class.
 *
 * @file CameraHandler.cc
 * @author Targed (ltklionel@gmail.com)
 * @date 2024-10-26
 *
 * @copyright Copyright MRDT 2024 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/handlers/CameraHandler.h"
#include "../../../TestingBase.hh"

/// \cond
#include <gmock/gmock.h>
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the CameraHandler
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
class CameraHandlerTests : public TestingBase<CameraHandlerTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new Camera Handler Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        CameraHandlerTests() { SetUp(); }

        /******************************************************************************
         * @brief Destroy the Camera Handler Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        ~CameraHandlerTests() { TearDown(); }

        /******************************************************************************
         * @brief Setup the Camera Handler Tests object.
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
         * @brief Teardown the Camera Handler Tests object.
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

// /******************************************************************************
//  * @brief Test for memory leaks
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
// ******************************************************************************/
// TEST_F(CameraHandlerTests, DoesNotLeak) {
//     CameraHandler* cameraHandler = new CameraHandler();
//     delete cameraHandler;
// }
//
// /******************************************************************************
//  * @brief Mock class for ZEDCam
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// class MockZEDCam : public ZEDCam {
//     // Mock methods for ZEDCam
// };

// /******************************************************************************
//  * @brief Mock class for BasicCam
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// class MockBasicCam : public BasicCam {
//     // Mock methods for BasicCam
// };

// /******************************************************************************
//  * @brief Mock class for RecordingHandler
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// class MockRecordingHandler : public RecordingHandler {
//     // Mock methods for RecordingHandler
// };

// /******************************************************************************
//  * @brief Test fixture for CameraHandler
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// class CameraHandlerTests : public ::testing::Test {
// protected:
//     CameraHandler* cameraHandler;
//     MockZEDCam* mockMainCam;
//     MockZEDCam* mockLeftCam;
//     MockZEDCam* mockRightCam;
//     MockBasicCam* mockGroundCam;
//     MockRecordingHandler* mockRecordingHandler;

//     void SetUp() override {
//         mockMainCam = new MockZEDCam();
//         mockLeftCam = new MockZEDCam();
//         mockRightCam = new MockZEDCam();
//         mockGroundCam = new MockBasicCam();
//         mockRecordingHandler = new MockRecordingHandler();
//         cameraHandler = new CameraHandler();
//         cameraHandler->m_pMainCam = mockMainCam;
//         cameraHandler->m_pLeftCam = mockLeftCam;
//         cameraHandler->m_pRightCam = mockRightCam;
//         cameraHandler->m_pGroundCam = mockGroundCam;
//         cameraHandler->m_pRecordingHandler = mockRecordingHandler;
//     }

//     void TearDown() override {
//         delete cameraHandler;
//         delete mockMainCam;
//         delete mockLeftCam;
//         delete mockRightCam;
//         delete mockGroundCam;
//         delete mockRecordingHandler;
//     }
// };

// /******************************************************************************
//  * @brief Test that the constructor initializes the members correctly
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(CameraHandlerTests, ConstructorInitializesMembers) {
//     CameraHandler handler;
//     EXPECT_EQ(handler.m_pMainCam, nullptr);
//     EXPECT_EQ(handler.m_pLeftCam, nullptr);
//     EXPECT_EQ(handler.m_pRightCam, nullptr);
//     EXPECT_EQ(handler.m_pGroundCam, nullptr);
//     EXPECT_EQ(handler.m_pRecordingHandler, nullptr);
// }

// /******************************************************************************
//  * @brief Test that the camera handler starts all cameras
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(CameraHandlerTests, StartAllCameras) {
//     EXPECT_CALL(*mockMainCam, Start()).Times(1);
//     EXPECT_CALL(*mockLeftCam, Start()).Times(1);
//     EXPECT_CALL(*mockRightCam, Start()).Times(1);
//     EXPECT_CALL(*mockGroundCam, Start()).Times(1);

//     cameraHandler->StartAllCameras();
// }

// /******************************************************************************
//  * @brief Test that the camera handler starts recording
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(CameraHandlerTests, StartRecording) {
//     EXPECT_CALL(*mockRecordingHandler, StartRecording()).Times(1);

//     cameraHandler->StartRecording();
// }

// /******************************************************************************
//  * @brief Test that the camera handler stops all cameras
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(CameraHandlerTests, StopAllCameras) {
//     EXPECT_CALL(*mockMainCam, Stop()).Times(1);
//     EXPECT_CALL(*mockLeftCam, Stop()).Times(1);
//     EXPECT_CALL(*mockRightCam, Stop()).Times(1);
//     EXPECT_CALL(*mockGroundCam, Stop()).Times(1);

//     cameraHandler->StopAllCameras();
// }

// /******************************************************************************
//  * @brief Test that the camera handler stops all cameras
//  *
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(CameraHandlerTests, StopRecording) {
//     EXPECT_CALL(*mockRecordingHandler, StopRecording()).Times(1);

//     cameraHandler->StopRecording();
// }

// /******************************************************************************
//  * @brief Test that the camera handler gets the ZED camera
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(CameraHandlerTests, GetZED) {
//     EXPECT_EQ(cameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam), mockMainCam);
//     EXPECT_EQ(cameraHandler->GetZED(CameraHandler::ZEDCamName::eFrameLeftCam), mockLeftCam);
//     EXPECT_EQ(cameraHandler->GetZED(CameraHandler::ZEDCamName::eFrameRightCam), mockRightCam);
// }

// /******************************************************************************
//  * @brief Test that the camera handler gets the basic camera
//  *
//  * @author Targed (ltklionel@gmail.com)
//  * @date 2024-10-26
//  ******************************************************************************/
// TEST_F(CameraHandlerTests, GetBasicCam) {
//     EXPECT_EQ(cameraHandler->GetBasicCam(CameraHandler::BasicCamName::eHeadGroundCam), mockGroundCam);
// }
