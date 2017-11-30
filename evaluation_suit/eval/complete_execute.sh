#! /bin/bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: ./execute.sh optimal_algo dataset_prefix";
    exit $?
fi 


ESTIMATORS=( "residual" "inverse" "photo" "point" "ssd" "ncc" )

for accuracy in "${ESTIMATORS[@]}"
do
	optimal=$1
	dataset=$2
	dir="${accuracy}_${optimal}"

	if [ ! -f "${accuracy}Report" ]; then
    		echo "Missing file ${accuracy}Report"
    		exit $?
	fi
	if [ ! -f "${dataset}_${optimal}" ]; then
    		echo "Missing file ${dataset}_${optimal}"
    		exit $?
	fi
	if [ ! -f manifoldReconstructor ]; then
    		echo "Missing file manifoldReconstructor"
    		exit $?
	fi

	mkdir "$dir"
	./system 10 "${accuracy}Report" "${dataset}_${optimal}" model.pov "${dir}/results.txt"
	cp images/* "${dir}/"
	mv poses_sfm_data_* "${dir}/"
	mv out_incremental* "${dir}/"
	rm images/zz_*
	rm zz_*
done

