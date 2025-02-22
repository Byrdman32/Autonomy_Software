####################################################################################################################
##                                               Autonomy Software                                                ##
##                                              24.05.00 - Build 001                                              ##
##                                             Mars Rover Design Team                                             ##
##                                      Copyright 2024 - All Rights Reserved                                      ##
####################################################################################################################

## Set CMake Minimum Version
cmake_minimum_required(VERSION 3.24.3)

## Enforce Using GCC 10 / G++ 10
execute_process(
    COMMAND gcc -dumpversion
    OUTPUT_VARIABLE GCC_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT GCC_VERSION VERSION_EQUAL "10.0")
    message(FATAL_ERROR "Only GCC 10.0 is allowed! Detected GCC version ${GCC_VERSION}.")
endif()

## C++ and CUDA Version
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

## Project Name and Version
project(Autonomy_Software VERSION 24.05.00 LANGUAGES CXX CUDA)

## CMake Policies
cmake_policy(SET CMP0146 OLD)    # Allows use of "FindCUDA" function
cmake_policy(SET CMP0153 OLD)    # Allows use of "Exec_Program" function

####################################################################################################################
##                                                    Options                                                     ##
####################################################################################################################

## Enable or Disable Simulation Mode
option(BUILD_SIM_MODE "Enable Simulation Mode" OFF)

## Enable or Disable Code Coverage Mode
option(BUILD_CODE_COVERAGE "Enable Code Coverage Mode" OFF)

## Enable or Disable Coverage Watch Mode
option(BUILD_COVERAGE_WATCH "Enable Code Coverage Watch Mode" ON)

## Enable or Disable Verbose Makefile
option(BUILD_VERBOSE_MODE "Enable Verbose Makefile" OFF)

## Enable or Disable Tests Mode
option(BUILD_TESTS_MODE "Enable Tests Mode" OFF)

## Enable or Disable Examples Mode
option(BUILD_EXAMPLES_MODE "Enable Examples Mode" OFF)

## Enable or Disable ZED SDK Shared Linking Mode
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

## Enable or Disable RoveComm Cross-Compile Mode
option(RC_CROSS_COMPILE "Cross-Compile RoveComm for Windows and Linux" OFF)

## Enable or Disable Printing All CMake Variables
option(LIST_ALL_VARS "Print all CMake Variables" OFF)

####################################################################################################################
##                                         Configuration Based on Options                                         ##
####################################################################################################################

message("-- Autonomy System Options:")

## Simulation Mode
if (BUILD_SIM_MODE)
    message("--   [x]: Sim Mode: Enabled")
    add_compile_definitions(__AUTONOMY_SIM_MODE__=1)
    set(EXE_NAME "${PROJECT_NAME}_Sim")
else()
    message("--   [ ]: Sim Mode: Disabled")
    add_compile_definitions(__AUTONOMY_SIM_MODE__=0)
    set(EXE_NAME "${PROJECT_NAME}")
endif()

## Code Coverage Mode
if (BUILD_CODE_COVERAGE)
    message("--   [x]: Code Coverage: Enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -fprofile-arcs -ftest-coverage --coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -fprofile-arcs -ftest-coverage --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
else()
    message("--   [ ]: Code Coverage: Disabled")
endif()

## Verbose Makefile
if (BUILD_VERBOSE_MODE)
    message("--   [x]: Verbose Makefile: Enabled")
    set(CMAKE_VERBOSE_MAKEFILE TRUE CACHE BOOL "Verbose output" FORCE)
else()
    message("--   [ ]: Verbose Makefile: Disabled")
endif()

## Build Unit and Integration Tests
if (BUILD_TESTS_MODE)
    message("--   [x]: Build Unit and Integration Tests: Enabled")
    include(CTest)
    enable_testing()
else()
    message("--   [ ]: Build Unit and Integration Tests: Disabled")
endif()

## Build Examples
if (BUILD_EXAMPLES_MODE)
    message("--   [x]: Build Examples: Enabled")
else()
    message("--   [ ]: Build Examples: Disabled")
endif()

## ZED SDK Shared Linking
if (LINK_SHARED_ZED)
    message("--   [x]: ZED SDK Shared Linking: Enabled")
    set(ZED_LIBS ${ZED_LIBRARIES} CUDA::toolkit)
else()
    message("--   [ ]: ZED SDK Shared Linking: Disabled")
    if (MSVC)
        message(FATAL_ERROR "ZED SDK static libraries not available on Windows")
    endif()
endif()

## RoveComm Cross-Compile
if (RC_CROSS_COMPILE)
    message("--   [x]: RoveComm Cross-Compile: Enabled")
else()
    message("--   [ ]: RoveComm Cross-Compile: Disabled")
    set(BUILD_WIN OFF CACHE BOOL "Disable Windows Cross-Compile Mode for Autonomy Software" FORCE)
endif()

## Print All CMake Variables
if (LIST_ALL_VARS)
    message("--   [x]: Print All CMake Variables: Enabled")
    
else()
    message("--   [ ]: Print All CMake Variables: Disabled")
endif()

####################################################################################################################
##                                           Dependencies and Libraries                                           ##
####################################################################################################################

# Configure BSThreadPool
add_compile_definitions(BS_THREAD_POOL_ENABLE_PAUSE=1)
add_compile_definitions(BS_THREAD_POOL_ENABLE_PRIORITY=1)
add_compile_definitions(BS_THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK=1)

## Find RoveComm
include_directories(external/rovecomm/src)
add_subdirectory(external/rovecomm)
add_compile_definitions(__ROVECOMM_LIBRARY_MODE__=1)

## Determine if shared or static libraries will be used.
if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

## Find Threads
find_package(Threads REQUIRED)

## Find OpenMP
find_package(OpenMP REQUIRED)

## Find Quill
find_package(quill REQUIRED)

## Find LibDataChannel
find_package(LibDataChannel REQUIRED)

## Fine nlohmann JSON.
find_package(nlohmann_json 3.2.0 REQUIRED)

## Find OpenSSL
find_package(OpenSSL REQUIRED)

## Find FFMPEG.
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavformat libavutil libswscale libswresample libavdevice libavfilter libpostproc SvtAv1Enc)
include_directories(${FFMPEG_INCLUDE_DIRS})

## Find Google Test
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

## Find Eigen3.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

## Find OpenCV.
find_package(OpenCV REQUIRED)

## Find Geographic Lib.
find_package(GeographicLib REQUIRED)

## Find ZEDSDK. Add as system package to supress library warnings.
find_package(ZED 4 REQUIRED)
include_directories(SYSTEM ${ZED_INCLUDE_DIRS})

## Find CUDA. Must match ZEDSDK version. Add as system package to supress library warnings.
find_package(CUDAToolkit ${ZED_CUDA_VERSION} REQUIRED)
include_directories(SYSTEM ${CUDAToolkit_INCLUDE_DIRS})

## Find PyTorch.
set(CAFFE2_USE_CUDNN "1")
set(TORCH_CUDA_ARCH_LIST "8.0 8.6 8.9 9.0")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

## Find flatbuffers. This is a dependency of tensorflow.
find_package(Flatbuffers REQUIRED)

## Find Abseil. This is a dependency of tensorflow.
find_package(absl REQUIRED)
set(ABSL_LIBRARY_DIRS absl::algorithm 
                    absl::base 
                    absl::debugging 
                    absl::flat_hash_map 
                    absl::flags 
                    absl::memory 
                    absl::meta 
                    absl::numeric 
                    absl::random_random 
                    absl::strings 
                    absl::synchronization 
                    absl::time 
                    absl::utility
)

## Find TensorflowLite.
find_package(TENSORFLOWLITE REQUIRED PATHS "external/tensorflow/")
include_directories(SYSTEM ${TENSORFLOWLITE_INCLUDE_DIRS})

## Find Libedgetpu. This is used for interfacing with all Coral devices.
find_package(LIBEDGETPU REQUIRED PATHS "external/tensorflow/")
include_directories(SYSTEM ${LIBEDGETPU_INCLUDE_DIRS})

## Define libraries for ZED. Optionally static or dynamic.
if (LINK_SHARED_ZED)
    set(ZED_LIBS ${ZED_LIBRARIES} CUDA::toolkit)
else()
    set(ZED_LIBS ${ZED_LIBRARIES_STATIC_RELEASE} CUDA::toolkit)
endif()

## Add special flag if compiling on aarch64.
if (CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
    add_definitions(-DJETSON_STYLE)
endif()

## Check if all variables should be listed.
if (LIST_ALL_VARS)
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
endif()

####################################################################################################################
##                                                Build Executable                                                ##
####################################################################################################################

## Search Project Directories for CPP Files
file(GLOB_RECURSE SRC           CONFIGURE_DEPENDS  "src/*.cpp")
file(GLOB_RECURSE Examples_SRC  CONFIGURE_DEPENDS  "examples/*/*.cpp")
file(GLOB_RECURSE External_SRC  CONFIGURE_DEPENDS  "external/src/*.cpp")
file(GLOB_RECURSE Tools_SRC     CONFIGURE_DEPENDS  "tools/*.cpp")

if (BUILD_EXAMPLES_MODE)
    add_executable(${EXE_NAME} ${External_SRC} ${SRC} ${Examples_SRC} ${Tools_SRC})
else()
    add_executable(${EXE_NAME} ${External_SRC} ${SRC} ${Tools_SRC})
endif()

## Compile Options
if (MSVC)
    target_compile_options(${EXE_NAME} PRIVATE /W4 /WX)
else()
    target_compile_options(${EXE_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

## Link Libraries
set(AUTONOMY_LIBRARIES  Threads::Threads
                        OpenMP::OpenMP_CXX
                        Eigen3::Eigen
                        RoveComm_CPP
                        quill::quill
                        ${OPENSSL_LIBRARIES}
                        PkgConfig::FFMPEG
                        LibDataChannel::LibDataChannel
                        nlohmann_json::nlohmann_json
                        ${OpenCV_LIBS}
                        ${GeographicLib_LIBRARIES}
                        ${ZED_LIBS}
                        ${TORCH_LIBRARIES}
                        flatbuffers::flatbuffers
                        ${ABSL_LIBRARY_DIRS}
                        ${TENSORFLOWLITE_LIBS}
                        ${LIBEDGETPU_LIBS}
                        gcov
)
target_link_libraries(${EXE_NAME} PRIVATE ${AUTONOMY_LIBRARIES})

####################################################################################################################
##                                             Installation and Tests                                             ##
####################################################################################################################

## Tests Mode
if (BUILD_TESTS_MODE)
    ## Find Unit and Integration Tests
    file(GLOB_RECURSE UTests_SRC            CONFIGURE_DEPENDS  "tests/Unit/src/*.cc")
    file(GLOB_RECURSE ITests_SRC            CONFIGURE_DEPENDS  "tests/Integration/src/*.cc")
    file(GLOB         Runner_SRC            CONFIGURE_DEPENDS  "tests/main.cc")

    ## Find Source Files Required for Tests
    file(GLOB_RECURSE Algorithms_SRC        CONFIGURE_DEPENDS  "src/algorithms/*.cpp")
    file(GLOB_RECURSE Drivers_SRC           CONFIGURE_DEPENDS  "src/drivers/*.cpp")
    file(GLOB_RECURSE Vision_SRC            CONFIGURE_DEPENDS  "src/vision/*.cpp")
    file(GLOB         Network_SRC           CONFIGURE_DEPENDS  "src/AutonomyNetworking.cpp")
    file(GLOB         Logging_SRC           CONFIGURE_DEPENDS  "src/AutonomyLogging.cpp")
    file(GLOB         Globals_SRC           CONFIGURE_DEPENDS  "src/AutonomyGlobals.cpp")   

    list(LENGTH UTests_SRC UnitTests_LEN)
    list(LENGTH ITests_SRC IntegrationTests_LEN)

    string(TIMESTAMP CURRENT_TIMESTAMP "%Y%m%d-%H%M%S")
    message("--   Current Timestamp: ${CURRENT_TIMESTAMP}")

    if (UnitTests_LEN GREATER 0)
        add_executable(${EXE_NAME}_UnitTests ${Runner_SRC} ${UTests_SRC} ${Algorithms_SRC} ${Drivers_SRC} ${Vision_SRC} ${Network_SRC} ${Logging_SRC} ${Globals_SRC})
        target_link_libraries(${EXE_NAME}_UnitTests GTest::gtest GTest::gtest_main ${AUTONOMY_LIBRARIES})

        foreach(test_file ${UTests_SRC})
            get_filename_component(test_name ${test_file} NAME_WE)
            add_test(NAME UTest_${test_name} COMMAND ${EXE_NAME}_UnitTests --gtest_filter=${test_name}Test*.* --timestamp=${CURRENT_TIMESTAMP})
            set_tests_properties(UTest_${test_name} PROPERTIES LABELS "UTest")
        endforeach()
    else()
        message("No Unit Tests!")
    endif()

    if (IntegrationTests_LEN GREATER 0)
        add_executable(${EXE_NAME}_IntegrationTests ${Runner_SRC} ${ITests_SRC} ${Algorithms_SRC} ${Drivers_SRC} ${Vision_SRC} ${Network_SRC} ${Logging_SRC} ${Globals_SRC})
        target_link_libraries(${EXE_NAME}_IntegrationTests GTest::gtest GTest::gtest_main ${AUTONOMY_LIBRARIES})

        foreach(test_file ${ITests_SRC})
            get_filename_component(test_name ${test_file} NAME_WE)
            add_test(NAME ITest_${test_name} COMMAND ${EXE_NAME}_IntegrationTests --gtest_filter=${test_name}Test*.* --timestamp=${CURRENT_TIMESTAMP})
            set_tests_properties(ITest_${test_name} PROPERTIES LABELS "iTest")
        endforeach()
    else()
        message("No Integration Tests!")
    endif()

    if (BUILD_CODE_COVERAGE AND BUILD_COVERAGE_WATCH)
        add_custom_target(run_coverage ALL
            COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
            COMMAND gcovr --root ${CMAKE_SOURCE_DIR} --xml-pretty --output Coverage.xml
            COMMAND lcov -c -d . -o coverage.info
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Running tests and generating coverage reports..."
            VERBATIM
        )

        # Ensure dependencies for the run_coverage target. Tests must be built first.
        if (UnitTests_LEN GREATER 0 AND IntegrationTests_LEN GREATER 0)
            add_dependencies(run_coverage ${EXE_NAME}_UnitTests ${EXE_NAME}_IntegrationTests)
        elseif (UnitTests_LEN GREATER 0)
            add_dependencies(run_coverage ${EXE_NAME}_UnitTests)
        elseif (IntegrationTests_LEN GREATER 0)
            add_dependencies(run_coverage ${EXE_NAME}_IntegrationTests)
        else()
            message("No Tests to Run! run_coverage target will not be created.")
            set_target_properties(run_coverage PROPERTIES EXCLUDE_FROM_ALL TRUE)
        endif()
    endif()
endif()

####################################################################################################################
##                                           Final Build Configuration                                            ##
####################################################################################################################

message("-- Final Autonomy Software Configuration Summary")
message("--   Executable Name: ${EXE_NAME}")
message("--   Simulation Mode: ${BUILD_SIM_MODE}")
message("--   Tests Mode: ${BUILD_TESTS_MODE}")
message("--   Examples Mode: ${BUILD_EXAMPLES_MODE}")
message("--   Code Coverage Mode: ${BUILD_CODE_COVERAGE}")
message("--   Verbose Makefile: ${BUILD_VERBOSE_MODE}")
message("--   ZED SDK Shared Linking Mode: ${LINK_SHARED_ZED}")
