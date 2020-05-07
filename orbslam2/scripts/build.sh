#!/usr/bin/env bash

cd ..

param_num=$#

if [ $param_num -eq 0 ]; then
    if [ -f "Vocabulary/ORBvoc.txt" ]; then
        echo "Vocabulary/ORBvoc.txt Exist"
    else
        echo "Download and Uncompress vocabulary ..."
        mkdir Vocabulary & cd Vocabulary
        if [ ! -f "ORBvoc.txt.tar.gz" ]; then
            wget https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/Vocabulary/ORBvoc.txt.tar.gz
        fi
        tar -xf ORBvoc.txt.tar.gz
        cd ..
    fi

    echo "Configuring and building ORB_SLAM2 ..."

    if [ ! -d "build" ]; then
        mkdir build
    fi
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j3

    echo
    echo "You can use command './`basename $0` clean' to delete build directories."
    echo

elif [ $param_num -eq 1 ]; then
    if [ "$1" = "clean" ]
    then
        echo "Deleting `pwd`/build"
        rm -rf build
    else
        echo "ERROR: input ERROR parameter!"
        exit 1
    fi

elif [ $param_num -ge 2 ]; then
    echo "ERROR: too many parameters!"
    exit 1
fi
