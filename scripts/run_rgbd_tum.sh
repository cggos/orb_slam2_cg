#!/usr/bin/env bash

cd ..

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml ../../datasets/rgbd_dataset_freiburg1_xyz ../../datasets/rgbd_dataset_freiburg1_xyz/associate.txt
