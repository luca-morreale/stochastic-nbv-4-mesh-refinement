#! /bin/bash

if [ -z $1 ] || [ -z $2 ];
then
    echo "Usage: ./compare_all.sh start_index end_index"
    exit $?
fi

start=$1
end=$2

accuracy=( "residual" "inverse" "photo" "point" "ncc" "ssd" );
optimal=( "gm" "mc" "gpso" "lpso" );

for acc in "${accuracy[@]}";
do
    for opt in "${optimal[@]}";
    do
        dir="${acc}_${opt}"
        if [ -d "$dir" ];
        then
            cd "$dir"
            cp ../comparator .
            ./comparator ../pcl_gt.asc "$start" "$end"
            cd ..
        fi
    done
done

find . -name "*.bin" | xargs rm

