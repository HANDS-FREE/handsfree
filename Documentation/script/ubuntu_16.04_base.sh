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

$install_command vim-common vim-doc vim-gtk vim-scripts build-essential bin86 g++ gcc libstdc++5 
$install_command classicmenu-indicator indicator-multiload tree cutecom meld
$install_command ack-grep cmake cmake-qt-gui git subversion mercurial yum openssh-server openssh-client
$install_command libncurses5 libncurses5-dev mesa-utils libglu1-mesa freeglut3 freeglut3-dev libxmu-dev libxmu-headers libcairo2 libcairo2-dev python-cairo

# gtk
$install_command libgtk2.0-dev

# install qt4
$install_command qtcreator libqt4-dev
$install_command libqt4-opengl-dev libqtwebkit-dev  libqt4-qt3support libqwtplot3d-qt4-dev qt4-dev-tools qt4-qtconfig  
#$install_command qt4-demos qt4-doc qt4-designer
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
