#!/bin/bash

cd ..
# Set your base directory (replace '/path/to/base/dir' with your actual base directory)
BASEDIR=$(pwd)

# Create directories
mkdir $BASEDIR/music-adapters_install
mkdir $BASEDIR/music-adapters_build

# Change to the build directory
cd $BASEDIR/music-adapters_build

# Run CMake with specified options
cmake -DCMAKE_INSTALL_PREFIX:PATH=$BASEDIR/music-adapters_install -DMUSIC_ROOT_DIR=$BASEDIR/MUSIC_install $BASEDIR/music-adapters

# Build using make with 4 parallel jobs
make -j

# Install the built files
make install


# Clean up the build directory
cd ..
rm -rf $BASEDIR/music-adapters_build

cd music-adapters

current_date_time=$(date "+%Y-%m-%d %H:%M:%S")
echo "Installation completed successfully at: $current_date_time"
