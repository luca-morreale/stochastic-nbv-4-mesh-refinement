#!/bin/bash

## Remove folders used to generate cloud of points
## in all sub directories

folders=$(ls -d */)

for folder in $folders; do
    cd $folder
    rm -R images
    rm -R matches
    rm -R out_Incremental_Reconstruction
    rm -R poses_sfm_data
    cd ..
done
