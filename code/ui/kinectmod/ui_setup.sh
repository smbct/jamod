#!/bin/bash

# echo "1:" $1 # ${CMAKE_CURRENT_SOURCE_DIR} 
# echo "2:" $2 # ${CMAKE_CURRENT_BINARY_DIR}
# echo "3:" $3 # ${CMAKE_INSTALL_PREFIX}

unzip -d $2/.. -u $3/JediAcademy/GameData/base/assets1.pk3 ui/setup.menu
mkdir -p $2/../ui_original && cp $2/../ui/setup.menu $2/../ui_original/setup.menu

unzip -d $2/.. -u $3/JediAcademy/GameData/base/assets1.pk3 ui/ingamesetup.menu
cp $2/../ui/ingamesetup.menu $2/../ui_original/ingamesetup.menu

rm -rf $2/../ui/*

# generate custom menu from patches
mkdir -p $1/ui/kinectmod/ui
patch -R -o $1/ui/kinectmod/ui/ingamesetup.menu $2/../ui_original/ingamesetup.menu < $1/ui/kinectmod/ingamesetup.menu.patch
patch -R -o $1/ui/kinectmod/ui/setup.menu $2/../ui_original/setup.menu < $1/ui/kinectmod/setup.menu.patch