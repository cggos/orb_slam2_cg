#!/usr/bin/env bash

param_num=$#

if [ $param_num -ne 1 ]; then
    echo "usage: ./run_stereo_euroc.sh <dataset_dir>"
else
    cd ..
    dataset_dir="$1"
    ./Examples/Stereo/stereo_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo/EuRoC.yaml \
    $dataset_dir/MH_01_easy/mav0/cam0/data \
    $dataset_dir/MH_01_easy/mav0/cam1/data \
    Examples/Stereo/EuRoC_TimeStamps/MH01.txt
fi
