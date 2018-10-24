#!/bin/sh
echo "Install GAUSS"
echo "Download and Install Qt 5.9.0"
wget http://download.qt.io/official_releases/qt/5.9/5.9.0/qt-opensource-linux-x64-5.9.0.run
chmod +x qt-opensource-linux-x64-5.9.0.run
sudo ./qt-opensource-linux-x64-5.9.0.run --platform minimal --verbose --script qt_script.qs

echo "Install Additional Packages"
sudo apt-get install build-essential xorg-dev cmake cmake-data libblas-dev liblapack-dev mesa-common-dev libglu1-mesa-dev -y libfontconfig1 libeigen3-dev

mkdir build
cd build

echo "Compile GAUSS"
cmake .. -DCMAKE_PREFIX_PATH=/opt/Qt5.9.0/5.9/gcc_64/lib/cmake -DEigen3_DIR=/usr/include/eigen3 -DCMAKE_BUILD_TYPE=Release
make -j 2 all 
                                                        
