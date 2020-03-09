#!/bin/sh
echo "Hello world"
ORIG_DIR=$(pwd)
echo "current directory: "$ORIG_DIR
mkdir $ORIG_DIR/external_libraries/pcl_1_9/build
mkdir $ORIG_DIR/external_libraries/pcl_1_9/install_dir
PCL_DIR=$ORIG_DIR/external_libraries/pcl_1_9/
cd $PCL_DIR/build
cmake -DCMAKE_INSTALL_PREFIX=$PCL_DIR/install_dir -DBUILD_CUDA=ON -DBUILD_GPU=ON ..
make -j4
make install

G2O_DIR=$ORIG_DIR/external_libraries/g2o_master/
mkdir $G2O_DIR/build
mkdir $G2O_DIR/install_dir
cd $G2O_DIR/build
cmake -DCMAKE_INSTALL_PREFIX=$G2O_DIR/install_dir ..
make -j4
make install



