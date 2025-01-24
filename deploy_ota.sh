#!/bin/bash

set -e

REMOTE_HOST=csiwiki
REMOTE_ASSETS_DIR=/home/wikijs/esp-ota-server/assets

# Generate a random short-sha version
VERSION=$(openssl rand -hex 4)

# Write the version into ota_version.h
echo "#define OTA_VERSION \"$VERSION\"" > main/include/ota_version.h

# Build and copy the binary to the remote assets directory
source activate
idf.py build
scp build/avionics-software-2024-2025.bin $REMOTE_HOST:$REMOTE_ASSETS_DIR/program.bin

# Write version.txt to the remote assets directory
ssh $REMOTE_HOST "echo $VERSION > $REMOTE_ASSETS_DIR/version.txt"

echo "Deployed OTA version: $VERSION"
