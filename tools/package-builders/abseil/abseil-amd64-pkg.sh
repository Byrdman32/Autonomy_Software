#!/bin/bash

# Set Working Directory
cd /tmp

# Install Variables
ABSEIL_VERSION="20230802.1"
FORCE_BUILD=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force|-f)
            FORCE_BUILD=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Define Package URL
FILE_URL="https://github.com/MissouriMRDT/Autonomy_Packages/raw/main/abseil/amd64/abseil_${ABSEIL_VERSION}_amd64.deb"

# Check if the file exists
if [[ "$FORCE_BUILD" == false ]] && curl --output /dev/null --silent --head --fail "$FILE_URL"; then
    echo "Package version ${ABSEIL_VERSION} already exists in the repository. Skipping build."
    echo "rebuilding_pkg=false" >> $GITHUB_OUTPUT
else
    echo "Package version ${ABSEIL_VERSION} does not exist in the repository. Building the package."
    echo "rebuilding_pkg=true" >> $GITHUB_OUTPUT
    
    # Delete Old Packages
    rm -rf /tmp/pkg
    rm -rf /tmp/abseil

    # Create Package Directory
    mkdir -p /tmp/pkg/abseil_${ABSEIL_VERSION}_amd64/usr/local
    mkdir -p /tmp/pkg/abseil_${ABSEIL_VERSION}_amd64/DEBIAN

    # Create Control File
    {
        echo "Package: abseil-mrdt"
        echo "Version: ${ABSEIL_VERSION}"
        echo "Maintainer: abseil"
        echo "Depends:"
        echo "Architecture: amd64"
        echo "Homepage: https://abseil.io/docs/cpp/guides/"
        echo "Description: A prebuilt version of Abseil. Made by the Mars Rover Design Team."
    } > /tmp/pkg/abseil_${ABSEIL_VERSION}_amd64/DEBIAN/control

    # Download Abseil
    git clone --depth 1 --branch ${ABSEIL_VERSION} https://github.com/abseil/abseil-cpp.git
    mkdir -p abseil-cpp/build && cd abseil-cpp/build

    # Build Abseil
    cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/pkg/abseil_${ABSEIL_VERSION}_amd64/usr/local \
    -D CMAKE_BUILD_TYPE=Release \
    -D ABSL_ENABLE_INSTALL=ON ..

    # Install Abseil
    make
    make install
    mkdir -p /tmp/pkg/abseil_${ABSEIL_VERSION}_amd64/usr/lib
    cp -r /tmp/pkg/abseil_${ABSEIL_VERSION}_amd64/usr/local/lib/libabsl* /tmp/pkg/abseil_${ABSEIL_VERSION}_amd64/usr/lib/

    # Cleanup Install
    cd ../..
    rm -rf abseil-cpp

    # Create Package
    dpkg --build /tmp/pkg/abseil_${ABSEIL_VERSION}_amd64

    # Create Package Directory
    mkdir -p /tmp/pkg/deb

    # Copy Package
    cp /tmp/pkg/abseil_${ABSEIL_VERSION}_amd64.deb /tmp/pkg/deb/abseil_${ABSEIL_VERSION}_amd64.deb
fi
