#!/usr/bin/env bash

cd ..

./bin/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml \
/home/cg/dev_sdb/datasets/TUM/RGBD-SLAM-Dataset/rgbd_dataset_freiburg1_room \
/home/cg/dev_sdb/datasets/TUM/RGBD-SLAM-Dataset/rgbd_dataset_freiburg1_room/associate.txt
