Introduction

Four main dependencies are required:
Flycapture2 to grab images from the Point Gray Camera
OpenCV for all the image processing magic
Boost for some background functions
TBB for parallel CPU

If you want to squeeze out the max performance out of your shiny intel CPU you should compile using intels icpc and icc high performance compilers. Academic use is free and can be requested at https://software.intel.com/en-us/academic/swdevtools
Grab a parallel studio XE edition and install it.

Prerequisites

1. Make sure the system is up to date:

sudo apt-get update
sudo apt-get upgrade

2. Install all the dependencies:

sudo apt-get -y install libopencv-dev build-essential cmake git libgtk2.0-dev pkg-config python-dev python-numpy python-tk libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff4-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libqt4-opengl-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils unzip libopenexr-dev default-jdk ant libvtk5-qt4-dev libx264-dev sphinx-common texlive-latex-extra libeigen3-dev yasm libx264-dev sphinx-common texlive-latex-extra libboost-dev libraw1394-11 libgtk2.0-0 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0

Installing Flycapture2

Flycapture2 is necessary for running point grey cameras. This is specifically for the Flea3 USB3.0 high definition camera. Flycapture2 is necessary for the custom eye-tracker. These instructions are for Linux based acquisition machines. The instructions assume a 64 bit installation. All dependencies are shared.

1. Make a directory in the /usr/local/src folder called flycapture and cd into it

sudo mkdir /usr/local/src/flycapture; cd /usr/local/src/flycapture

2. Copy the download file from brains software archive and unzip it.

get the latest flycapture SDK from http://www.ptgrey.com/support/downloads
sudo tar zxvf flycapture2-XXX-amd64-pkg.tgz (XXX is the version number)
cd flycapture2-XXX-amd64

3. Add raw1394 to /etc/modules file. To do this, open the modules file via

sudo vim /etc/modules

and add the following line to the end

raw1394

4. Now just install flycapture using the install script:

sudo bash install_flycapture.sh

5. You will be prompted to install all SDK packages, choose yes

6. You will then be asked if you want to add a udev entry to allow access to users, choose yes

7. Type your USERNAME at the next prompt to add yourself to the user group

8. Confirm that your USERNAME is okay, choose yes.

9. Choose yes at next prompt to add your USERNAME to group pgrimaging

10. Choose yes at next prompt to restart the udev daemon.

Congrats!

Installing OpenCV

Here are the instructions for installing OpenCV on Ubuntu boxes

3. Get the OpenCV 3.X source code:

http://opencv.org/downloads.html

4. Set up the build:

sudo mkdir build
cd build

for a build with standard gcc g++ do:

CXXFLAGS="-Ofast -march=native" CFLAGS=$CXXFLAGS cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON ENABLE_FAST_MATH=ON ..

for a build with intels compilers do:

source source /opt/intel/bin/compilervars.sh intel64

CXX=icpc CC=icc CXXFLAGS="-Ofast -march=native" CFLAGS=$CXXFLAGS cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON ENABLE_FAST_MATH=ON ..

If you are using intel make sure that your cmake checkout shows the compiler was found and is going to be used!
Same goes for TBB!

5. Now compile and build (this can take a while):

sudo make -j4
sudo make install

6. Configure OpenCV by creating and editing the following file:

sudo vi /etc/ld.so.conf.d/opencv.conf

Add the following line to the end of the file:

/usr/local/lib

7. Run the following line to configure the library:

sudo ldconfig

8. Now we have to edit one more file:

sudo vi /etc/bash.bashrc

Add the following to the end of that file and save

PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH

9. Now close the terminal, open a new one, then reboot the system.

Voila - OpenCV is ready to use
