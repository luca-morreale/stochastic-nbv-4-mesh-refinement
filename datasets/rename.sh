#!/bin/bash

## Copy all png files in the subfolders that respect the
## patter set* in a folder called images

folders=$(ls -d set*)

mkdir "images"

for folder in $folders; do
    cd $folder
    for file in *.png; do
        cp "${file}" "../images/${folder}_${file}"
    done
    cd ..
done
