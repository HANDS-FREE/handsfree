#!/bin/bash

# install command
install_command="sudo apt-get install -y"

################################################################################
# ia32 support librays
################################################################################
echo "ia32 support librays"
$install_command ia32-libs*

################################################################################
# building tools & librarys
################################################################################
echo "building tools & librarys"

$install_command vim-common vim-doc vim-gtk vim-scripts build-essential bin86 kernel-package g++ gcc libstdc++5 

$install_command exuberant-ctags cscope rcs manpages-dev glibc-doc manpages-posix manpages-posix-dev

$install_command ack-grep cmake cmake-qt-gui git subversion mercurial yum openssh-server openssh-client


$install_command libncurses5 libncurses5-dev mesa-utils libglu1-mesa freeglut3 freeglut3-dev libxmu-dev libxmu-headers libcairo2 libcairo2-dev python-cairo

# gtk
$install_command libgtk2.0-dev

# install qt4
$install_command libqt4-dev qt4-demos qt4-doc qt4-designer 
$install_command libqt4-opengl-dev libqtwebkit-dev
$install_command libqt4-qt3support libqwtplot3d-qt4-dev qt4-dev-tools qt4-qtconfig libqt4-opengl-dev
$install_command qtcreator
$install_command python-qt4 python-qt4-doc python-qt4-gl
$install_command libqglviewer-dev libqglviewer2

# install qt5
$install_command qt5-default

# install opengl
$install_command build-essential libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev

# install SDL
$install_command libsdl2-dev

# numerical calculation
$install_command libeigen3-dev libeigen3-doc
$install_command libsuitesparse-dev 

# other
$install_command lib3ds-dev
$install_command libgtk2.0-dev
$install_command libgtkglext1 libgtkglext1-dev
$install_command libgstreamer1.0-dev
$install_command libdc1394-22-dev
$install_command libv4l-dev 
$install_command libjpeg-dev
$install_command libpng12-dev

#$install_command xfsprogs
$install_command p7zip-full p7zip-rar unrar lbzip2 pigz

$install_command ethtool
$install_command atop

# exFAT
$install_command exfat-fuse

