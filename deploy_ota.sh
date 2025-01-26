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
scp build/avionics-software-2024-2025.bin $REMOTE_HOST:$REMOTE_ASSETS_DIR/program.bin.tmp

# Atomically switch the program.bin file and update version.txt
ssh $REMOTE_HOST "mv $REMOTE_ASSETS_DIR/program.bin.tmp $REMOTE_ASSETS_DIR/program.bin && \
    echo $VERSION > $REMOTE_ASSETS_DIR/version.txt"

echo "Deployed OTA version: $VERSION"
