#!/usr/bin/env bash

cd ..

./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml ../../datasets/MH_01_easy/mav0/cam0/data ../../datasets/MH_01_easy/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/MH01.txt
