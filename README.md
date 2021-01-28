# NAKED
Autonomous Sanitizer RosBot

## Setting up Orbbec Astra
  1. Download the latest version of Orbbec OpenNI SDK
  ```
$ cd Linux/OpenNI-Linux-x64-2.3.0.63/
$ sudo ./install.sh
$ source OpenNIDevEnvironment
  ```
  2. Run the following commands to verify that OpenNI library can be found
  ```
$ echo $OPENNI2_INCLUDE
/home/user/OpenNI_2.3.0.63/Linux/OpenNI-Linux-x64-2.3.0.63/Include
$ echo $OPENNI2_REDIST
/home/user/OpenNI_2.3.0.63/Linux/OpenNI-Linux-x64-2.3.0.63/Redist
  ```
  3. Install OpenCV Pre-Requisites
  ```
[compiler] sudo apt-get install build-essential
[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
[optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
  ```
  4. Clone OpenCV 
  ```
cd ~/opencv
mkdir build
cd build
  ```
  5. Run CMake on Source
  ```
cmake -DWITH_OPENNI2=ON -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..

  ```
  6. Build OpenCV from Source
  ```
make -j7 # runs 7 jobs in parallel
  ```
  7. Install to System
  ```
sudo make install
  ```
