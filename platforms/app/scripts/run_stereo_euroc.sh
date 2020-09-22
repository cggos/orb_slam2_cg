#!/usr/bin/env bash

PROJECT_ROOT="../../../"

param_num=$#

if [ $param_num -ne 1 ]; then
    echo "usage: ./run_stereo_euroc.sh <dataset_dir>"
else
    dataset_dir="$1"
    ../bin/stereo_euroc \
    $PROJECT_ROOT/Vocabulary/ORBvoc.txt \
    ../Stereo/EuRoC.yaml \
    $dataset_dir/MH_01_easy/mav0/cam0/data \
    $dataset_dir/MH_01_easy/mav0/cam1/data \
    ../Stereo/EuRoC_TimeStamps/MH01.txt
fi
