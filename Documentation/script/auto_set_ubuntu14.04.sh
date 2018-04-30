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

$install_command vim-common vim-doc vim-gtk vim-scripts build-essential bin86 kernel-package g++ gcc libstdc++5 gcc-4.6 g++-4.6 gcc-4.4 g++-4.4

$install_command exuberant-ctags cscope rcs manpages-dev glibc-doc manpages-posix manpages-posix-dev

$install_command ack-grep cmake cmake-qt-gui git subversion mercurial yum openssh-server openssh-client

$install_command libncurses5 libncurses5-dev mesa-utils libglu1-mesa freeglut3 freeglut3-dev libxmu-dev libxmu-headers libcairo2 libcairo2-dev python-cairo

# gtk
$install_command libgtk2.0-dev

# install qt4
$install_command libqt4-core libqt4-dev libqt4-gui qt4-doc qt4-designer qtcreator
$install_command libqt4-opengl-dev libqtwebkit-dev libqt4-qt3support libqwtplot3d-qt4-0 libqwtplot3d-qt4-dev qt4-dev-tools qt4-qtconfig libqt4-opengl-dev python-qt4 python-qt4-doc python-qt4-gl libqglviewer-dev libqglviewer2

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

# nfs
$install_command nfs-kernel-server nfs-common

$install_command xfsprogs
$install_command p7zip-full p7zip-rar unrar lbzip2 pigz

# archivement mount
$install_command libarchive-dev libfuse-dev libfuse2

# install music player
$install_command audacious audacious-dev audacious-plugins

# mplayer
$install_command mplayer mplayer-doc mencoder smplayer mplayer-fonts

# ffmpeg
$install_command ffmpeg
$install_command libavcodec54 libavcodec-dev libavdevice53 libavdevice-dev libavfilter3 libavfilter-dev libavformat54 libavformat-dev libavutil-dev libavutil52 libswscale-dev libswscale2 libavresample-dev

# exFAT
$install_command exfat-fuse

