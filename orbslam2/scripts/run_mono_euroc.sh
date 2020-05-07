#!/usr/bin/env bash

cd ..

./bin/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml ../../datasets/MH_01_easy/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/MH01.txt
