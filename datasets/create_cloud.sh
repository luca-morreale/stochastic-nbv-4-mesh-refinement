#! /bin/bash

## Delete all folder used previously by this script,
## if any. Then recreate them and exectute all the
## necessary steps to generate the cloud of points.
## The intrinsic parameters are read from the file
## K.txt, it must be in the current working directory.
## It is possible to choose of which set(s) of images
## create the cloud of points. It is necessary to
## comment the line(s) that issue the command remove
## containing the relative folder's name.


filename="K.txt"
read -r intrinsic_params < "$filename"

rm -R images
rm -R matches
rm -R out_Incremental_Reconstruction
rm -R poses_sfm_data

../rename.sh

rm images/set1_*
rm images/set2*
rm images/set3*
rm images/set4*
rm images/set5*
rm images/set6*
rm images/set7*
rm images/set8*
rm images/set9*
rm images/set10*
rm images/set11*
rm images/set12*
rm images/set13*
rm images/set14*
rm images/set15*

openMVG_main_SfMInit_ImageListing -i images -d /usr/local/share/openMVG/sensor_width_camera_database.txt -o matches -k "$intrinsic_params"

openMVG_main_ComputeFeatures -i matches/sfm_data.json -o matches/ -m AKAZE_FLOAT
openMVG_main_ComputeMatches -i matches/sfm_data.json -o matches/
openMVG_main_IncrementalSfM -i matches/sfm_data.json -m matches/ -o out_Incremental_Reconstruction

mkdir poses_sfm_data

openMVG_main_ComputeStructureFromKnownPoses -i out_Incremental_Reconstruction/sfm_data.bin -o poses_sfm_data/sfm_data.json -m matches/
