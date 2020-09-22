#!/usr/bin/env bash

PROJECT_ROOT="../../../"

dataset_dir="$1"

../bin/mono_tum $PROJECT_ROOT/Vocabulary/ORBvoc.txt ../Monocular/TUM1.yaml $dataset_dir
