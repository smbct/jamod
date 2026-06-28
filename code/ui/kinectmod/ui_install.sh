#!/bin/bash

echo "first param $1" # source dir
echo "second param $2" # build dir
echo "third param $3" # install dir

# copy menu files to build dir
cp $1/ui/kinectmod/ui/* $2/../ui

# install more recent files
cd $2/../
zip $3/JediAcademy/GameData/base/assets1.pk3 ./ui/ingamesetup.menu
zip $3/JediAcademy/GameData/base/assets1.pk3 ./ui/setup.menu
echo "test"

# generate patch files
diff $2/../ui/ingamesetup.menu $2/../ui_original/ingamesetup.menu > $1/ui/kinectmod/ingamesetup.menu.patch || true
diff $2/../ui/setup.menu $2/../ui_original/setup.menu > $1/ui/kinectmod/setup.menu.patch || true

