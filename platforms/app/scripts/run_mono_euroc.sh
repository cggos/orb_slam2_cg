#!/usr/bin/env bash

PROJECT_ROOT="../../../"

dataset_dir="$1"

../bin/mono_euroc \
    $PROJECT_ROOT/Vocabulary/ORBvoc.txt \
    ../Monocular/EuRoC.yaml \
    $dataset_dir/MH_01_easy/mav0/cam0/data \
    ../Monocular/EuRoC_TimeStamps/MH01.txt
